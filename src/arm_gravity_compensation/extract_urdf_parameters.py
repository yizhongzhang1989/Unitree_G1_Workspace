#!/usr/bin/env python3
"""Extract every URDF link inertial into a calibration JSON file."""

import argparse

from arm_gravity_compensation.parameter_store import ParameterStore


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", help="source URDF file")
    parser.add_argument("output", help="output JSON file")
    parser.add_argument(
        "--force", action="store_true",
        help="replace an existing parameter file")
    arguments = parser.parse_args()

    store = ParameterStore(arguments.output)
    document = store.initialize(arguments.urdf, force=arguments.force)
    inertial_count = sum(
        link["inertial"] is not None for link in document["links"].values())
    print("Wrote %s" % store.path)
    print("Links: %d, inertials: %d" % (
        len(document["links"]), inertial_count))
    print("URDF SHA256: %s" % document["source_urdf"]["sha256"])


if __name__ == "__main__":
    main()