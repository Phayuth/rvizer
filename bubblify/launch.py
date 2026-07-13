import sys
from pathlib import Path
from bubblify import BubblifyApp


def main():
    urdf_path = Path("/rb3_730es_u/rb3_730es_u.urdf")

    try:
        app = BubblifyApp(
            robot_name="robot",
            urdf_path=urdf_path,
            show_collision=True,
            port=8080,
        )
        app.run()

    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
