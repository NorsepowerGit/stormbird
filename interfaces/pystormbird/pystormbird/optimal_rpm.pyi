
def compute_optimal_rpm_grid(
    sim_json_string: str,
    conditions: list[tuple[float, float, float]],
    rpm_combinations: list[list[float]],
    power_curve_coeffs: list[list[float]],
    fallback_rpm: float,
    height_correction_factor: float,
    lateral_force_limit: float,
    resultant_force_limit: float,
    rs_power_limit: float,
    main_engine_efficiency: float,
    minimum_savings: float,
    n_workers: int,
) -> list[tuple[
    list[float],   # best_rpms (n_rotors)
    list[float],   # fwd_forces kN (n_rotors)
    list[float],   # lat_forces kN (n_rotors)
    float,         # total_thrust
    float,         # total_power_generated
    float,         # total_power_required
    float,         # total_power_savings
    list[float],   # per_rotor_power_generated
    list[float],   # per_rotor_power_required
    list[float],   # per_rotor_power_savings
]]: ...
