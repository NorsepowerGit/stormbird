import argparse

import numpy as np
import pandas as pd

import plotly.graph_objects as go
from plotly.subplots import make_subplots

from tillig_model import tillig_lift_coefficient_vec, tillig_drag_coefficient_vec


def get_rotor_columns(df: pd.DataFrame) -> list[str]:
    """Extract rotor sail column names (RSx_RPM) from the dataframe."""
    return [col for col in df.columns if col.startswith('RS') and col.endswith('_RPM')]


def calculate_rotor_positions(n_rotors: int, diameter: float, spacing_factor: float = 2.0) -> tuple[list[float], list[float]]:
    """
    Calculate rotor sail positions lined up in a straight line along the y-axis.
    
    Rotors are spaced based on diameter with a spacing factor to avoid interference.
    """
    spacing = diameter * spacing_factor
    
    x_locations = [0.0] * n_rotors
    y_locations = []
    
    center_offset = (n_rotors - 1) * spacing / 2.0
    
    for i in range(n_rotors):
        y_locations.append(i * spacing - center_offset)
    
    return x_locations, y_locations


def calculate_forces_from_parquet(
    parquet_path: str,
    diameter: float,
    height: float,
    density: float = 1.225
) -> pd.DataFrame:
    """
    Calculate forward and lateral forces for rotor sails from parquet data.

    Uses the Tillig empirical model to calculate forces directly from spin ratio
    without requiring a solver. Uses vectorized operations for performance.

    The Tillig model gives lift and drag coefficients normalised by rotor projected area D×H.
    Forces are calculated as: F = C × q × D × H, where q = 0.5 × ρ × V² is the dynamic pressure.
    RPM sign is handled by mirroring CL: negative RPM produces negative (mirrored) lift.

    Args:
        parquet_path: Path to parquet file with AWS_ACT, AWA_ACT, RSx_RPM columns
        diameter: Rotor sail diameter in meters
        height: Rotor sail height in meters
        density: Air density in kg/m^3 (default: 1.225)

    Returns:
        DataFrame with original data plus calculated forces for each rotor
    """
    df = pd.read_parquet(parquet_path)

    rotor_columns = get_rotor_columns(df)

    n_rows = len(df)
    df = df[df[rotor_columns].abs().max(axis=1) > 30]
    print(f"Filtered from {n_rows} rows to {len(df)} rows based on RPM > 30")

    n_rotors = len(rotor_columns)

    if n_rotors == 0:
        raise ValueError("No rotor sail columns (RSx_RPM) found in the parquet file")

    print(f"Found {n_rotors} rotor sail(s): {rotor_columns}")

    x_locations, y_locations = calculate_rotor_positions(n_rotors, diameter)

    print(f"Rotor positions (x, y): {list(zip(x_locations, y_locations))}")

    # Reference area = diameter × height (Tillig model, normalised by projected rotor area)
    reference_length = height
    print(f"Using reference area: D × H = {diameter:.1f} × {reference_length:.1f} = {diameter * reference_length:.1f} m²")

    aws = df['AWS_ACT'].values
    awa_deg = df['AWA_ACT'].values
    awa_rad = np.radians(awa_deg)

    # The Tillig polynomial is valid for spin ratios roughly 0–5.
    # Clamp to this range to avoid catastrophic extrapolation at very low wind speeds.
    SR_MAX = 5.0
    MIN_AWS = 1.0  # m/s — below this the rotor aerodynamics are negligible
    valid_mask = aws >= MIN_AWS
    n_low_wind = np.sum(aws < MIN_AWS)
    if n_low_wind > 0:
        print(f"Warning: {n_low_wind} rows have AWS < {MIN_AWS} m/s — forces set to zero for those rows")

    for rotor_idx, rotor_col in enumerate(rotor_columns):
        rpm = df[rotor_col].values

        # Spin ratio = tangential velocity / wind speed (always positive for coefficient lookup)
        # RPM can be negative (rotor spinning in reverse direction), so use absolute value.
        # The sign of RPM determines the sign of CL: negative RPM mirrors the lift direction.
        spin_ratio = np.zeros_like(rpm, dtype=float)
        spin_ratio[valid_mask] = (np.pi * np.abs(rpm[valid_mask]) * diameter) / (60.0 * aws[valid_mask])
        spin_ratio = np.clip(spin_ratio, 0.0, SR_MAX)
        rpm_sign = np.sign(rpm)

        cl = rpm_sign * tillig_lift_coefficient_vec(spin_ratio)
        cd = tillig_drag_coefficient_vec(spin_ratio)

        # Dynamic pressure: q = 0.5 × ρ × V²
        dynamic_pressure = 0.5 * density * aws**2

        # Tillig coefficients are normalised by rotor projected area: reference area = D × H
        # Total force: F = C × q × D × H
        lift_force = cl * dynamic_pressure * diameter * reference_length
        drag_force = cd * dynamic_pressure * diameter * reference_length

        # Resolve into ship-fixed coordinates
        # AWA = 0° means wind from ahead (headwind)
        # Forward force: positive = thrust (forward), negative = drag (aft)
        # Lateral force: positive = to starboard, negative = to port
        #
        # Wind vector in ship coordinates:
        # - Wind FROM ahead (AWA=0°): wind vector points aft (-x direction)
        # - Wind FROM starboard (AWA=90°): wind vector points to port (-y direction)
        #
        # Drag acts in direction of wind (resistance)
        # Lift acts perpendicular to wind (Magnus effect)
        #
        # For positive RPM (top moving starboard) and wind from starboard (AWA=90°):
        # - Lift generates forward thrust (+x)
        # - Drag acts to port (-y)
        
        fwd_force = drag_force * np.cos(awa_rad) - lift_force * np.sin(awa_rad)
        lat_force = lift_force * np.cos(awa_rad) + drag_force * np.sin(awa_rad)

        # Convert to kN to match actual data (RSx_FWD is in kN)
        df[f'{rotor_col.replace("_RPM", "_FWD_FORCE")}'] = fwd_force / 1000.0
        df[f'{rotor_col.replace("_RPM", "_LAT_FORCE")}'] = lat_force / 1000.0

    return df


def plot_force_timeline(df: pd.DataFrame, rotor_columns: list[str], diameter: float, height: float) -> go.Figure:
    """
    Create a timeline plot comparing actual forward forces (RSx_FWD in kN) 
    with calculated forward forces for each rotor sail.
    
    Args:
        df: DataFrame with actual and calculated force columns
        rotor_columns: List of rotor column names (RSx_RPM)
        diameter: Rotor diameter (for display)
        height: Rotor height (for display)
    
    Returns:
        Plotly figure object
    """
    n_rotors = len(rotor_columns)
    
    fig = make_subplots(
        rows=n_rotors,
        cols=1,
        subplot_titles=[f"Rotor {col.replace('RS', '').replace('_RPM', '')}" for col in rotor_columns],
        shared_xaxes=True,
        vertical_spacing=0.05
    )
    
    time_col = 'timestamp' if 'timestamp' in df.columns else None
    x_values = df[time_col] if time_col else df.index
    
    for i, rotor_col in enumerate(rotor_columns):
        row = i + 1
        
        actual_col = rotor_col.replace('_RPM', '_FWD')
        calculated_col = rotor_col.replace('_RPM', '_FWD_FORCE')
        
        if actual_col in df.columns:
            actual_force = df[actual_col]  # Already in kN
            fig.add_trace(
                go.Scatter(
                    x=x_values,
                    y=actual_force,
                    mode='lines',
                    name=f'{rotor_col.replace("_RPM", "")} Actual',
                    line=dict(color='blue', width=1),
                    # legendgroup=f'rotor_{i}',
                    showlegend=(row == 1)
                ),
                row=row,
                col=1
            )

        if calculated_col in df.columns:
            calculated_force = df[calculated_col]  # Now also in kN
            fig.add_trace(
                go.Scatter(
                    x=x_values,
                    y=calculated_force,
                    mode='lines',
                    name=f'{rotor_col.replace("_RPM", "")} Calculated',
                    line=dict(color='red', width=1, dash='dash'),
                    # legendgroup=f'rotor_{i}',
                    showlegend=(row == 1)
                ),
                row=row,
                col=1
            )

        fig.update_yaxes(title_text="Force [kN]", row=row, col=1)
    
    fig.update_xaxes(title_text="Time step", row=n_rotors, col=1)
    
    fig.update_layout(
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        title_text=f"Rotor Sail Forward Forces (D={diameter}m, H={height}m)",
        title_x=0.5
    )
    
    return fig


def plot_force_scatter(df: pd.DataFrame, rotor_columns: list[str], diameter: float, height: float) -> go.Figure:
    """
    Create scatter plots comparing actual vs calculated forward forces for each rotor sail.

    Args:
        df: DataFrame with actual and calculated force columns
        rotor_columns: List of rotor column names (RSx_RPM)
        diameter: Rotor diameter (for display)
        height: Rotor height (for display)

    Returns:
        Plotly figure object
    """

    n_rotors = len(rotor_columns)

    fig = make_subplots(
        rows=1,
        cols=n_rotors,
        subplot_titles=[f"Rotor {col.replace('RS', '').replace('_RPM', '')}" for col in rotor_columns],
        shared_yaxes=True,
        horizontal_spacing=0.05
    )

    for i, rotor_col in enumerate(rotor_columns):
        col = i + 1

        actual_col = rotor_col.replace('_RPM', '_FWD')
        calculated_col = rotor_col.replace('_RPM', '_FWD_FORCE')

        if actual_col in df.columns and calculated_col in df.columns:
            actual_force = df[actual_col]  # Already in kN
            calculated_force = df[calculated_col]  # Also in kN

            fig.add_trace(
                go.Scatter(
                    x=actual_force,
                    y=calculated_force,
                    mode='markers',
                    name=f'{rotor_col.replace("_RPM", "")}',
                    marker=dict(color='green', size=5, opacity=0.5),
                    showlegend=(i == 0)
                ),
                row=1,
                col=col
            )

        fig.update_xaxes(title_text="Actual Force [kN]", row=1, col=col)

    fig.update_yaxes(title_text="Calculated Force [kN]", row=1, col=1)

    fig.update_layout(
        showlegend=True,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        title_text=f"Actual vs Calculated Forward Forces (D={diameter}m, H={height}m)",
        title_x=0.5
    )

    return fig


def main():
    parser = argparse.ArgumentParser(
        description="Calculate forward and lateral forces for rotor sails from parquet data"
    )
    parser.add_argument(
        "parquet_path",
        type=str,
        help="Path to parquet file containing AWS_ACT, AWA_ACT, and RSx_RPM columns"
    )
    parser.add_argument(
        "--diameter",
        type=float,
        default=5.0,
        help="Rotor sail diameter in meters (default: 5.0)"
    )
    parser.add_argument(
        "--height",
        type=float,
        default=35.0,
        help="Rotor sail height in meters (default: 35.0)"
    )
    parser.add_argument(
        "--density",
        type=float,
        default=1.225,
        help="Air density in kg/m^3 (default: 1.225)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Output path for results parquet file (default: print to console)"
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show timeline plot comparing actual vs calculated forces"
    )
    parser.add_argument(
        "--plot-output",
        type=str,
        default=None,
        help="Save plot to HTML file"
    )

    args = parser.parse_args()

    print(f"Processing parquet file: {args.parquet_path}")
    print(f"Rotor sail diameter: {args.diameter} m")
    print(f"Rotor sail height: {args.height} m")
    print(f"Air density: {args.density} kg/m^3")
    print()

    result_df = calculate_forces_from_parquet(
        parquet_path=args.parquet_path,
        diameter=args.diameter,
        height=args.height,
        density=args.density
    )
    
    if args.output:
        result_df.to_parquet(args.output, index=False)
        print(f"\nResults saved to: {args.output}")
    else:
        print("\nResults preview:")
        print(result_df.head())
    
    if args.plot or args.plot_output:
        rotor_columns = get_rotor_columns(result_df)
        fig = plot_force_timeline(result_df, rotor_columns, args.diameter, args.height)
        
        if args.plot_output:
            fig.write_html(args.plot_output)
            print(f"\nPlot saved to: {args.plot_output}")
        
        if args.plot:
            fig.show()

        # fig = plot_force_scatter(result_df, rotor_columns, args.diameter, args.height)
        #
        # if args.plot_output:
        #     scatter_output = args.plot_output.replace('.html', '_scatter.html')
        #     fig.write_html(scatter_output)
        #     print(f"\nScatter plot saved to: {scatter_output}")
        #
        # if args.plot:
        #     fig.show()


if __name__ == "__main__":
    main()
