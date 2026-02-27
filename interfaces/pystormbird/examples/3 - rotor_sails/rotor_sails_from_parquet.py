import argparse
import sys
import os

import numpy as np
import pandas as pd

import plotly.graph_objects as go
from plotly.subplots import make_subplots

from stormbird_setup.direct_setup.spatial_vector import SpatialVector
from stormbird_setup.direct_setup.section_models import SectionModel, RotatingCylinder
from stormbird_setup.direct_setup.line_force_model import LineForceModelBuilder, WingBuilder
from stormbird_setup.direct_setup.lifting_line.simulation_builder import SimulationBuilder, QuasiSteadySettings
from stormbird_setup.direct_setup.lifting_line.solver import Linearized
from stormbird_setup.direct_setup.lifting_line.wake import QuasiSteadyWakeSettings, SymmetryCondition
from stormbird_setup.direct_setup.lifting_line.velocity_corrections import VelocityCorrections, VelocityCorrectionType
from pystormbird.lifting_line import Simulation


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MIN_AWS = 1.0   # m/s — below this wind speed forces are set to zero
NR_SECTIONS = 32
MAX_INDUCED_VELOCITY_RATIO = 2.0


def get_rotor_columns(df: pd.DataFrame) -> list[str]:
    """Extract rotor sail column names (RSx_RPM) from the dataframe."""
    return [col for col in df.columns if col.startswith('RS') and col.endswith('_RPM')]


def calculate_rotor_positions(
    n_rotors: int,
    diameter: float,
    spacing_factor: float = 2.0,
) -> tuple[list[float], list[float]]:
    """
    Calculate rotor sail positions along the centreline (y-axis).

    Rotors are evenly spaced along y with spacing = diameter × spacing_factor.
    All rotors share x = 0 (on the ship centreline).
    """
    spacing = diameter * spacing_factor
    center_offset = (n_rotors - 1) * spacing / 2.0
    x_locations = [0.0] * n_rotors
    y_locations = [i * spacing - center_offset for i in range(n_rotors)]
    return x_locations, y_locations


def get_section_model() -> SectionModel:
    """
    Rotor sail section model using the Tillig CL data, tuned to match 3-D CFD results.
    Identical to the model used in setup.py.
    """
    return SectionModel(
        model=RotatingCylinder(
            spin_ratio_data=[0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0],
            cl_data=[0.0, 1.22, 2.56, 5.93, 8.87, 9.56, 10.22, 12.0, 13.00],
        )
    )


def build_simulation(
    x_locations: list[float],
    y_locations: list[float],
    diameter: float,
    height: float,
    density: float,
) -> Simulation:
    """
    Build and return a pystormbird Simulation for the given rotor layout.

    The simulation is constructed once and reused for every time step:
    only the freestream velocity and the per-rotor RPM (internal state) change
    between rows.

    Coordinate system (stormbird / ship frame):
      +x  = forward (bow direction)
      +y  = starboard
      +z  = up

    Wake: QuasiSteadyWakeSettings with Z-symmetry (sea-surface reflection),
    matching the settings in setup.py.
    """
    chord_vector = SpatialVector(x=diameter)

    wing_builders = []
    for x, y in zip(x_locations, y_locations):
        wing_builders.append(
            WingBuilder(
                section_points=[
                    SpatialVector(x=x, y=y, z=0.0),
                    SpatialVector(x=x, y=y, z=height),
                ],
                chord_vectors=[chord_vector, chord_vector],
                section_model=get_section_model(),
                non_zero_circulation_at_ends=(True, False),
            )
        )

    line_force_model = LineForceModelBuilder(
        nr_sections=NR_SECTIONS,
        density=density,
    )
    for wb in wing_builders:
        line_force_model.add_wing_builder(wb)

    sim_builder = SimulationBuilder(
        line_force_model=line_force_model,
        simulation_settings=QuasiSteadySettings(
            solver=Linearized(),
            wake=QuasiSteadyWakeSettings(symmetry_condition=SymmetryCondition.Z),
        ),
    )
    sim_builder.simulation_settings.solver.velocity_corrections = VelocityCorrections(
        type=VelocityCorrectionType.MaxInducedVelocityMagnitudeRatio,
        value=MAX_INDUCED_VELOCITY_RATIO,
    )

    return Simulation(sim_builder.to_json_string())


def calculate_forces_from_parquet(
    parquet_path: str,
    diameter: float,
    height: float,
    density: float = 1.225,
) -> pd.DataFrame:
    """
    Calculate forward and lateral forces for rotor sails from parquet data.

    Uses the pystormbird lifting-line solver with QuasiSteadyWakeSettings so
    that aerodynamic interaction between rotors (wake effects) is accounted for.
    The simulation geometry is built once; per-row calls only update the
    freestream velocity and rotor RPM.

    Rows where AWS < MIN_AWS are skipped (forces left as zero) to avoid
    numerical issues at near-calm conditions.

    Coordinate conventions
    ----------------------
    Parquet / ship frame:
      AWA  : 0° = wind from ahead, positive clockwise (so 90° = wind from stbd)
      Forces: positive forward = thrust; positive lateral = to starboard

    Stormbird frame (same as ship frame):
      +x   : forward (bow)
      +y   : starboard
      Wind direction angle θ is measured as the direction the wind *blows toward*,
      CCW from +x.  Conversion: θ = (180 + AWA) % 360

    RPM sign:
      Negative RPM in the parquet data → top of rotor moves forward at beam reach
      → generates forward thrust.  Passed to stormbird as rev/s = RPM / 60.

    Args:
        parquet_path : Path to parquet file with AWS_ACT, AWA_ACT, RSx_RPM columns.
        diameter     : Rotor sail diameter [m].
        height       : Rotor sail height [m].
        density      : Air density [kg/m³] (default 1.225).

    Returns:
        DataFrame with original data plus RSx_FWD_FORCE and RSx_LAT_FORCE columns [kN].
    """
    df = pd.read_parquet(parquet_path)

    rotor_columns = get_rotor_columns(df)
    if not rotor_columns:
        raise ValueError("No rotor sail columns (RSx_RPM) found in the parquet file")

    n_rows = len(df)
    df = df[df[rotor_columns].abs().max(axis=1) > 30]
    print(f"Filtered from {n_rows} rows to {len(df)} rows based on |RPM| > 30")

    n_rotors = len(rotor_columns)
    print(f"Found {n_rotors} rotor sail(s): {rotor_columns}")

    x_locations, y_locations = calculate_rotor_positions(n_rotors, diameter)
    print(f"Rotor positions (x, y): {list(zip(x_locations, y_locations))}")

    # ------------------------------------------------------------------
    # Build the stormbird simulation (done once for the whole dataset)
    # ------------------------------------------------------------------
    print("Building stormbird lifting-line simulation ...")
    simulation = build_simulation(x_locations, y_locations, diameter, height, density)
    n_freestream_pts = len(simulation.get_freestream_velocity_points())
    print(f"Simulation built ({n_freestream_pts} freestream velocity points, "
          f"{NR_SECTIONS} sections per rotor)")

    # ------------------------------------------------------------------
    # Pre-allocate output arrays
    # ------------------------------------------------------------------
    fwd_forces = {col: np.zeros(len(df)) for col in rotor_columns}
    lat_forces = {col: np.zeros(len(df)) for col in rotor_columns}

    aws_arr  = df['AWS_ACT'].values
    awa_arr  = df['AWA_ACT'].values
    rpm_arrs = {col: df[col].values for col in rotor_columns}

    # ------------------------------------------------------------------
    # Row-by-row evaluation
    # ------------------------------------------------------------------
    n_valid = 0
    n_low_wind = 0
    n_total = len(df)

    print(f"Evaluating {n_total} time steps ...")
    for i in range(n_total):
        aws = aws_arr[i]
        awa = awa_arr[i]

        if aws < MIN_AWS:
            n_low_wind += 1
            continue  # leave forces as zero

        # Convert AWA to stormbird wind direction:
        # AWA=0   (from ahead) → wind blows aft    (-x) → stormbird θ = 180°
        # AWA=90  (from stbd)  → wind blows to port(-y) → stormbird θ = 270°
        sb_wind_dir_rad = np.radians((180.0 + awa) % 360.0)
        vx = aws * np.cos(sb_wind_dir_rad)
        vy = aws * np.sin(sb_wind_dir_rad)
        freestream = [[vx, vy, 0.0]] * n_freestream_pts

        # Rev/s for each rotor: positive RPM → positive rev/s
        rev_per_s = [rpm_arrs[col][i] / 60.0 for col in rotor_columns]
        simulation.set_section_models_internal_state(rev_per_s)

        result = simulation.do_step(time=float(i), time_step=1.0, freestream_velocity=freestream)

        for r_idx, col in enumerate(rotor_columns):
            force = result.integrated_forces[r_idx].total   # [Fx, Fy, Fz] in N
            fwd_forces[col][i] = force[0] / 1000.0          # N → kN, +x = forward
            lat_forces[col][i] = force[1] / 1000.0          # N → kN, +y = starboard

        n_valid += 1

        if (i + 1) % 1000 == 0 or (i + 1) == n_total:
            print(f"  {i + 1}/{n_total} rows processed ({n_low_wind} skipped for low wind)")

    print(f"Done. {n_valid} rows evaluated, {n_low_wind} rows skipped (AWS < {MIN_AWS} m/s).")

    # ------------------------------------------------------------------
    # Write results back into the dataframe
    # ------------------------------------------------------------------
    for col in rotor_columns:
        df[col.replace('_RPM', '_FWD_FORCE')] = fwd_forces[col]
        df[col.replace('_RPM', '_LAT_FORCE')] = lat_forces[col]

    return df


def plot_force_timeline(
    df: pd.DataFrame,
    rotor_columns: list[str],
    diameter: float,
    height: float,
) -> go.Figure:
    """
    Timeline plot comparing actual (RSx_FWD) vs calculated forward forces for each rotor.
    """
    n_rotors = len(rotor_columns)

    fig = make_subplots(
        rows=n_rotors,
        cols=1,
        subplot_titles=[f"Rotor {col.replace('RS', '').replace('_RPM', '')}" for col in rotor_columns],
        shared_xaxes=True,
        vertical_spacing=0.05,
    )

    x_values = df.index

    for i, rotor_col in enumerate(rotor_columns):
        row = i + 1
        actual_col    = rotor_col.replace('_RPM', '_FWD')
        calculated_col = rotor_col.replace('_RPM', '_FWD_FORCE')

        if actual_col in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=x_values,
                    y=df[actual_col],
                    mode='lines',
                    name=f'{rotor_col.replace("_RPM", "")} Actual',
                    line=dict(color='blue', width=1),
                    showlegend=(row == 1),
                ),
                row=row, col=1,
            )

        if calculated_col in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=x_values,
                    y=df[calculated_col],
                    mode='lines',
                    name=f'{rotor_col.replace("_RPM", "")} Calculated',
                    line=dict(color='red', width=1, dash='dash'),
                    showlegend=(row == 1),
                ),
                row=row, col=1,
            )

        fig.update_yaxes(title_text="Force [kN]", row=row, col=1)

    fig.update_xaxes(title_text="Time", row=n_rotors, col=1)
    fig.update_layout(
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        title_text=f"Rotor Sail Forward Forces (D={diameter}m, H={height}m, wake-coupled)",
        title_x=0.5,
    )
    return fig


def plot_force_scatter(
    df: pd.DataFrame,
    rotor_columns: list[str],
    diameter: float,
    height: float,
) -> go.Figure:
    """
    Scatter plot: actual (x-axis) vs calculated (y-axis) forward force for each rotor.
    """
    n_rotors = len(rotor_columns)

    fig = make_subplots(
        rows=1,
        cols=n_rotors,
        subplot_titles=[f"Rotor {col.replace('RS', '').replace('_RPM', '')}" for col in rotor_columns],
        shared_yaxes=True,
        horizontal_spacing=0.05,
    )

    for i, rotor_col in enumerate(rotor_columns):
        col_idx = i + 1
        actual_col    = rotor_col.replace('_RPM', '_FWD')
        calculated_col = rotor_col.replace('_RPM', '_FWD_FORCE')

        if actual_col in df.columns and calculated_col in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df[actual_col],
                    y=df[calculated_col],
                    mode='markers',
                    name=rotor_col.replace('_RPM', ''),
                    marker=dict(color='green', size=5, opacity=0.5),
                    showlegend=True,
                ),
                row=1, col=col_idx,
            )
        fig.update_xaxes(title_text="Actual Force [kN]", row=1, col=col_idx)

    fig.update_yaxes(title_text="Calculated Force [kN]", row=1, col=1)
    fig.update_layout(
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        title_text=f"Actual vs Calculated Forward Forces (D={diameter}m, H={height}m)",
        title_x=0.5,
    )
    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Calculate rotor sail forces from parquet data using the "
                    "pystormbird lifting-line solver with wake interaction."
    )
    parser.add_argument(
        "parquet_path",
        type=str,
        help="Path to parquet file with AWS_ACT, AWA_ACT, and RSx_RPM columns",
    )
    parser.add_argument("--diameter", type=float, default=5.0,
                        help="Rotor sail diameter [m] (default: 5.0)")
    parser.add_argument("--height",   type=float, default=35.0,
                        help="Rotor sail height [m] (default: 35.0)")
    parser.add_argument("--density",  type=float, default=1.225,
                        help="Air density [kg/m³] (default: 1.225)")
    parser.add_argument("--output",   type=str,   default=None,
                        help="Save results to this parquet file path")
    parser.add_argument("--plot",     action="store_true",
                        help="Show interactive timeline plot")
    parser.add_argument("--plot-output", type=str, default=None,
                        help="Save timeline plot as HTML file")
    parser.add_argument("--scatter",  action="store_true",
                        help="Show actual vs calculated scatter plot")

    args = parser.parse_args()

    print(f"Processing: {args.parquet_path}")
    print(f"Rotor diameter: {args.diameter} m  |  height: {args.height} m  |  density: {args.density} kg/m³")
    print()

    result_df = calculate_forces_from_parquet(
        parquet_path=args.parquet_path,
        diameter=args.diameter,
        height=args.height,
        density=args.density,
    )

    if args.output:
        result_df.to_parquet(args.output)
        print(f"\nResults saved to: {args.output}")
    else:
        print("\nResults preview:")
        print(result_df[[c for c in result_df.columns if 'FORCE' in c or c in ('AWS_ACT', 'AWA_ACT')]].head())

    rotor_columns = get_rotor_columns(result_df)

    if args.plot or args.plot_output:
        fig = plot_force_timeline(result_df, rotor_columns, args.diameter, args.height)
        if args.plot_output:
            fig.write_html(args.plot_output)
            print(f"\nTimeline plot saved to: {args.plot_output}")
        if args.plot:
            fig.show()

    if args.scatter:
        fig = plot_force_scatter(result_df, rotor_columns, args.diameter, args.height)
        fig.show()


if __name__ == "__main__":
    main()
