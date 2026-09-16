"""Where run logs go: <workspace>/logs when running from the source tree (symlink install),
otherwise ~/.ros/robofetch_logs."""
import os


def default_log_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    ws = os.path.normpath(os.path.join(here, "..", "..", ".."))
    if os.path.isdir(os.path.join(ws, "src")):
        return os.path.join(ws, "logs")
    return os.path.join(os.path.expanduser("~"), ".ros", "robofetch_logs")
