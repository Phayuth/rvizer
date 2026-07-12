from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

# Add the project root to Python path so we can import bubblify
sys.path.insert(0, str(Path(__file__).parent.parent))

from bubblify import BubblifyApp


def main():
    """Main entry point for the Bubblify MWE."""
    parser = argparse.ArgumentParser(
        description="Interactive URDF spherization tool using Viser",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --robot panda
  %(prog)s --robot ur10 --show_collision
  %(prog)s --urdf_path /path/to/custom_robot.urdf
  %(prog)s --robot atlas_drc --port 8081
        """,
    )

    parser.add_argument(
        "--robot", type=str, default="panda", help="Robot name from robot_descriptions package (default: panda)"
    )

    parser.add_argument("--urdf_path", type=Path, help="Path to custom URDF file (overrides --robot if specified)")

    parser.add_argument("--show_collision", action="store_true", help="Show collision meshes in addition to visual meshes")

    parser.add_argument("--port", type=int, default=8080, help="Viser server port (default: 8080)")

    args = parser.parse_args()

    # Validate arguments
    if args.urdf_path is not None and not args.urdf_path.exists():
        print(f"L Error: URDF file not found: {args.urdf_path}")
        sys.exit(1)

    # Welcome message
    print("<� Welcome to Bubblify - Interactive URDF Spherization Tool!")
    print("=" * 60)

    if args.urdf_path is not None:
        print(f"=� Loading custom URDF: {args.urdf_path}")
        robot_name = args.urdf_path.stem
    else:
        print(f"> Loading robot: {args.robot}")
        robot_name = args.robot

    print(f"< Server will start on port {args.port}")
    print(f"=A  Show collision meshes: {'Yes' if args.show_collision else 'No'}")
    print()

    try:
        # Create and run the application
        app = BubblifyApp(
            robot_name=robot_name if args.urdf_path is None else "custom",
            urdf_path=args.urdf_path,
            show_collision=args.show_collision,
            port=args.port,
        )

        print("🎮 GUI Controls:")
        print("  • Use 'Robot Controls' to configure joints and visibility")
        print("  • Use 'Sphere Editor' to add and edit collision spheres")
        print("  • Use 'Export' to save your spherization")
        print()
        print("💡 Tips:")
        print("  • Select a link, then add spheres to it")
        print("  • Use the 3D transform gizmo to position spheres")
        print("  • Click on spheres in the 3D view to select them")
        print("  • Toggle mesh visibility and adjust sphere opacity for focus")
        print("  • Export YAML for quick save/load, URDF for final use")
        print()

        # Run the application
        app.run()

    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
        sys.exit(0)
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("💡 Make sure you have installed the required dependencies:")
        print("   pip install viser yourdfpy trimesh robot-descriptions")
        sys.exit(1)
    except Exception as e:
        print(f"❌ An error occurred: {e}")
        print("💡 Check your robot name or URDF path and try again")
        sys.exit(1)


if __name__ == "__main__":
    main()