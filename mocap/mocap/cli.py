"""python -m mocap <command>. Each work package adds its commands here."""
import argparse


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python -m mocap")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("probe", help="inspect the videos (fps, sync, resolution) and write contact sheets")
    p.add_argument("paths", nargs="*", help="videos or folders; default: everything in config/rig.yaml")
    p.add_argument("--no-timestamps", action="store_true", help="skip reading frame timestamps (faster)")
    p.add_argument("--no-sheets", action="store_true", help="skip the contact sheet images")
    p.add_argument("--out", help="output folder (default out/probe)")

    args = parser.parse_args(argv)
    if args.command == "probe":
        from mocap import probe
        probe.run(args.paths, timestamps=not args.no_timestamps, sheets=not args.no_sheets, out_dir=args.out)
