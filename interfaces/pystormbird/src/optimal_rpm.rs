// Copyright (C) 2024, NTNU
// Author: Norse Python Utils contributors
// License: GPL v3.0

//! Pure-Rust optimal RPM grid computation for rotor sails.
//!
//! `compute_optimal_rpm_grid` replaces the Python-level grid loop in
//! `StormbirdOptimalRPMCalculator.set_inputs`.  All aerodynamic calls are
//! executed inside Rust (optionally in parallel with Rayon), bypassing
//! the Python interpreter for the hot inner loop.
//!
//! ## Two-phase optimisation
//!
//! The dominant cost in the original implementation was rebuilding the N×N
//! horseshoe-vortex influence matrix (O(N²) vortex math) for *every* RPM
//! combination.  Because the wake direction only depends on (AWA, AWS) and
//! not on RPM, the matrix is identical for all combos at the same condition.
//!
//! This version splits the work into two phases per condition:
//!
//! 1. **`prepare_quasi_steady_wake`** — called *once* per (AWA, AWS) condition.
//!    Computes the geometry and builds the N×N wake-influence matrix.
//! 2. **`solve_linearized_integrated_forces`** — called *once* per RPM combo.
//!    Sets the RPM state, runs the Linearized solver against the cached matrix,
//!    and returns only `[Fx, Fy, Fz]` per rotor without constructing the full
//!    `SimulationResult`.
//!
//! This eliminates the O(N²) rebuild from the inner loop, replacing it with
//! just the O(N³) Gaussian elimination + O(N) force integration per combo.

use pyo3::prelude::*;
use rayon::prelude::*;

use stormbird::lifting_line::simulation::Simulation as SimulationRust;
use stormath::spatial_vector::SpatialVector;

// ─── small polynomial helper ─────────────────────────────────────────────────

/// Evaluate a polynomial given as ascending-power coefficients (numpy.polynomial.Polynomial
/// convention: coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 + …).
#[inline]
fn eval_poly(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut power = 1.0;
    for &c in coeffs {
        result += c * power;
        power *= x;
    }
    result
}

// ─── exported Python function ─────────────────────────────────────────────────

/// Compute the optimal RPM combination for every (SOG, AWA, AWS) condition.
///
/// Arguments
/// ---------
/// sim_json_string : str
///     JSON produced by `SimulationBuilder.to_json_string()`.  One
///     `Simulation` is cloned from this for every Rayon worker thread.
///
/// conditions : list[tuple[float, float, float]]
///     Each element is ``(sog_m_s, awa_deg, aws_m_s)``.
///
/// rpm_combinations : list[list[float]]
///     Shape ``(n_combos, n_rotors)``.  Already pre-filtered by the caller
///     (fallback RPM guaranteed to be present).
///
/// power_curve_coeffs : list[list[float]]
///     Per-rotor polynomial coefficients in *ascending-power* order
///     (numpy.polynomial.Polynomial convention).
///
/// fallback_rpm : float
/// height_correction_factor : float
/// lateral_force_limit : float       (kN, use f64::INFINITY to disable)
/// resultant_force_limit : float     (kN, use f64::INFINITY to disable)
/// rs_power_limit : float            (kW, use f64::INFINITY to disable)
/// main_engine_efficiency : float
/// minimum_savings : float           (kW)
/// n_workers : int                   (≤ 1 means single-threaded)
///
/// Returns
/// -------
/// list of per-condition results, each a tuple:
///   (
///     best_rpms: list[float],          # length n_rotors
///     fwd_forces: list[float],         # kN per rotor (for the best combo)
///     lat_forces: list[float],         # kN per rotor
///     total_thrust: float,
///     total_power_generated: float,
///     total_power_required: float,
///     total_power_savings: float,
///     per_rotor_power_generated: list[float],
///     per_rotor_power_required: list[float],
///     per_rotor_power_savings: list[float],
///   )
#[pyfunction]
#[pyo3(signature = (
    sim_json_string,
    conditions,
    rpm_combinations,
    power_curve_coeffs,
    fallback_rpm,
    height_correction_factor,
    lateral_force_limit,
    resultant_force_limit,
    rs_power_limit,
    main_engine_efficiency,
    minimum_savings,
    n_workers,
))]
pub fn compute_optimal_rpm_grid(
    sim_json_string: String,
    conditions: Vec<(f64, f64, f64)>,        // (sog, awa, aws)
    rpm_combinations: Vec<Vec<f64>>,          // (n_combos, n_rotors)
    power_curve_coeffs: Vec<Vec<f64>>,        // (n_rotors, n_coeffs)
    fallback_rpm: f64,
    height_correction_factor: f64,
    lateral_force_limit: f64,
    resultant_force_limit: f64,
    rs_power_limit: f64,
    main_engine_efficiency: f64,
    minimum_savings: f64,
    n_workers: usize,
) -> PyResult<Vec<(
    Vec<f64>,   // best_rpms
    Vec<f64>,   // fwd_forces  (kN)
    Vec<f64>,   // lat_forces  (kN)
    f64,        // total_thrust
    f64,        // total_power_generated
    f64,        // total_power_required
    f64,        // total_power_savings
    Vec<f64>,   // per_rotor_power_generated
    Vec<f64>,   // per_rotor_power_required
    Vec<f64>,   // per_rotor_power_savings
)>> {
    let n_rotors = power_curve_coeffs.len();
    let n_combos = rpm_combinations.len();

    if n_combos == 0 {
        return Err(pyo3::exceptions::PyValueError::new_err(
            "rpm_combinations is empty",
        ));
    }

    // Build one base simulation outside the thread pool; clone into each worker.
    let base_sim = SimulationRust::new_from_string(&sim_json_string)
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;

    let n_freestream_pts = base_sim.line_force_model.nr_span_lines();

    // We expose `n_workers` but Rayon uses a global thread pool. We build a
    // custom thread pool so the caller can control parallelism.
    let actual_workers = if n_workers < 1 { 1 } else { n_workers };

    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(actual_workers)
        .build()
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(format!("{e:?}")))?;

    // Release the GIL for the entire computation.
    let results: Vec<_> = Python::attach(|py| {
        py.detach(|| {
            pool.install(|| {
                conditions
                    .par_iter()
                    .map(|&(sog, awa, aws)| {
                        // Each Rayon worker thread gets its own cloned simulation.
                        let mut sim = base_sim.clone();
                        let aws_corrected = aws * height_correction_factor;

                        // ── Phase 1: build wake matrix once for this (AWA, AWS) ──────────
                        // AWA=0 (from ahead) → stormbird θ=180°; AWA=90 (from stbd) → θ=270°.
                        let sb_wind_dir_rad = ((180.0 + awa) % 360.0).to_radians();
                        let vx = aws_corrected * sb_wind_dir_rad.cos();
                        let vy = aws_corrected * sb_wind_dir_rad.sin();
                        let freestream: Vec<SpatialVector> =
                            vec![SpatialVector::new(vx, vy, 0.0); n_freestream_pts];

                        sim.prepare_quasi_steady_wake(&freestream);

                        // ── Phase 2: sweep all RPM combos ──────────────────────────────
                        let mut best_total_savings = f64::NEG_INFINITY;
                        let mut best_idx = 0usize;
                        // Cache forces for the best combo so we don't re-solve.
                        let mut best_fwd = vec![0.0f64; n_rotors];
                        let mut best_lat = vec![0.0f64; n_rotors];

                        for (combo_idx, rpms) in rpm_combinations.iter().enumerate() {
                            let rev_per_s: Vec<f64> =
                                rpms.iter().map(|&r| r / 60.0).collect();

                            // Solve with cached wake matrix — only O(N³) + O(N) work.
                            let forces = sim.solve_linearized_integrated_forces(
                                &rev_per_s,
                                &freestream,
                            );

                            // Per-rotor validity and savings
                            let mut combo_valid = true;
                            let mut total_savings = 0.0;
                            let mut fwd_tmp = vec![0.0f64; n_rotors];
                            let mut lat_tmp = vec![0.0f64; n_rotors];

                            for r in 0..n_rotors {
                                let fx = forces[r][0] / 1000.0; // N → kN
                                let fy = forces[r][1] / 1000.0;
                                fwd_tmp[r] = fx;
                                lat_tmp[r] = fy;

                                let is_fallback = (rpms[r] - fallback_rpm).abs() < 1e-6;

                                // Force validity
                                let res = (fx * fx + fy * fy).sqrt();
                                let force_ok = is_fallback
                                    || (fy.abs() <= lateral_force_limit
                                        && res <= resultant_force_limit);

                                // Power
                                let power_gen = fx * sog / main_engine_efficiency;
                                let power_req =
                                    eval_poly(&power_curve_coeffs[r], rpms[r].abs());

                                let power_ok =
                                    is_fallback || power_req <= rs_power_limit;

                                let savings = power_gen - power_req;
                                let savings_ok = is_fallback || savings >= minimum_savings;

                                if !force_ok || !power_ok || !savings_ok {
                                    combo_valid = false;
                                    break;
                                }

                                total_savings += savings;
                            }

                            if combo_valid && total_savings > best_total_savings {
                                best_total_savings = total_savings;
                                best_idx = combo_idx;
                                best_fwd = fwd_tmp;
                                best_lat = lat_tmp;
                            }
                        }

                        // ── Compute final per-rotor power metrics ─────────────────────
                        let best_rpms = rpm_combinations[best_idx].clone();
                        let mut total_thrust = 0.0f64;
                        let mut total_power_gen = 0.0f64;
                        let mut total_power_req = 0.0f64;
                        let mut per_rotor_power_gen = vec![0.0f64; n_rotors];
                        let mut per_rotor_power_req = vec![0.0f64; n_rotors];
                        let mut per_rotor_savings = vec![0.0f64; n_rotors];

                        for r in 0..n_rotors {
                            let thrust = best_fwd[r];
                            let power_gen = thrust * sog / main_engine_efficiency;
                            let power_req =
                                eval_poly(&power_curve_coeffs[r], best_rpms[r].abs());

                            per_rotor_power_gen[r] = power_gen;
                            per_rotor_power_req[r] = power_req;
                            per_rotor_savings[r] = power_gen - power_req;

                            total_thrust += thrust;
                            total_power_gen += power_gen;
                            total_power_req += power_req;
                        }

                        (
                            best_rpms,
                            best_fwd,
                            best_lat,
                            total_thrust,
                            total_power_gen,
                            total_power_req,
                            total_power_gen - total_power_req,
                            per_rotor_power_gen,
                            per_rotor_power_req,
                            per_rotor_savings,
                        )
                    })
                    .collect()
            })
        })
    });

    Ok(results)
}

// ─── submodule registration ───────────────────────────────────────────────────

#[pymodule]
pub fn optimal_rpm(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(compute_optimal_rpm_grid, m)?)?;
    Ok(())
}
