import os
import shutil
import subprocess
from typing import Optional
from pathlib import Path


def _run_dialog_command(cmd: list[str]) -> Optional[str]:
    """Run a folder dialog command and return the selected path if successful."""
    try:
        proc = subprocess.run(
            cmd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception:
        return None

    if proc.returncode != 0:
        return None

    selected = (proc.stdout or "").strip()
    if not selected:
        return None
    return os.path.abspath(os.path.expanduser(selected))


def os_select_folder(
    initial_dir: Optional[str] = None, title: str = "Select Folder"
) -> str:
    """Open a native folder picker without Tkinter.

    Falls back to terminal input when no GUI picker is available.
    Returns an absolute path; if the user cancels, returns the original initial_dir
    (or current working directory if initial_dir is not set).
    """
    start_dir = os.path.abspath(os.path.expanduser(initial_dir or os.getcwd()))

    # Linux desktop dialogs
    if shutil.which("zenity"):
        selected = _run_dialog_command(
            [
                "zenity",
                "--file-selection",
                "--directory",
                "--title",
                title,
                "--filename",
                start_dir + os.sep,
            ]
        )
        if selected:
            return selected

    if shutil.which("kdialog"):
        selected = _run_dialog_command(
            [
                "kdialog",
                "--getexistingdirectory",
                start_dir,
                "--title",
                title,
            ]
        )
        if selected:
            return selected

    # macOS dialog
    if shutil.which("osascript"):
        script = (
            'set pickedFolder to choose folder with prompt "'
            + title.replace('"', '\\"')
            + '" default location POSIX file "'
            + start_dir.replace('"', '\\"')
            + '"\n'
            + "POSIX path of pickedFolder"
        )
        selected = _run_dialog_command(["osascript", "-e", script])
        if selected:
            return selected

    # Terminal fallback
    print(f"{title}: GUI picker not found. Enter folder path manually.")
    print(f"Press Enter to keep: {start_dir}")
    typed = input("Folder path: ").strip()
    if not typed:
        return start_dir

    typed_path = os.path.abspath(os.path.expanduser(typed))
    return typed_path


def os_list_directory(directory) -> str:
    try:
        entries = sorted(
            directory.iterdir(),
            key=lambda entry: (not entry.is_dir(), entry.name.lower()),
        )
    except Exception as exc:
        content = f"Could not read directory: {exc}"
        return

    if not entries:
        content = "_(empty)_"
        return

    lines = []
    lines.append("```")
    for entry in entries:
        lines.append(f"{entry.name}/" if entry.is_dir() else entry.name)
    lines.append("```")
    content = "\n".join(lines)

    return content


if __name__ == "__main__":
    # Example usage
    selected_folder = os_select_folder(initial_dir="~", title="Select a folder")
    print(f"Selected folder: {selected_folder}")

    p = Path(selected_folder)

    file_content = os_list_directory(p)
    print(file_content)
