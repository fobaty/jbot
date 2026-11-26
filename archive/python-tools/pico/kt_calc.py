#!/usr/bin/env python3
"""Estimate units_per_nm and the tmax setting from a stall-torque measurement."""

import argparse

GRAVITY = 9.81


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--units", type=float, required=True,
                        help="test command units that just hold the load")
    parser.add_argument("--mass", type=float, required=True,
                        help="hanging mass in kg")
    parser.add_argument("--radius", type=float, required=True,
                        help="wheel radius in m")
    args = parser.parse_args()

    load_torque_nm = args.mass * GRAVITY * args.radius
    units_per_nm = args.units / load_torque_nm
    tmax_nm = 1000.0 / units_per_nm

    print(f"load torque:   {load_torque_nm:.3f} Nm")
    print(f"units_per_nm:  {units_per_nm:.1f}")
    print(f"suggested tmax:{tmax_nm:8.2f} Nm")
    print(f"vs current 5.0 Nm: {tmax_nm / 5.0:.2f}x")


if __name__ == "__main__":
    main()
