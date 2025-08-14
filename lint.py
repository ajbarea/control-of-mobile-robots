#!/usr/bin/env python3
"""
Simple linting and testing script for Control of Mobile Robots project.

This script runs the following tools in sequence:
1. isort   - Import sorting check
2. black   - Python code formatting
3. flake8  - Python linting
4. pytest  - Python tests
"""

import subprocess
import sys
from typing import List, Tuple


def run_command(command: List[str], description: str) -> Tuple[bool, str]:
    """
    Run a shell command and return success status and output.

    Args:
        command: List of command parts to execute
        description: Human-readable description of what the command does

    Returns:
        Tuple of (success: bool, output: str)
    """
    print(f"\n[RUNNING] {description}...")
    print(f"   Command: {' '.join(command)}")

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
        )

        if result.returncode == 0:
            print(f"[SUCCESS] {description} completed successfully!")
            if result.stdout.strip():
                print(f"   Output:\n{result.stdout.strip()}")
            return True, result.stdout
        else:
            print(f"[FAILED] {description} failed!")
            output = (result.stdout.strip() + "\n" + result.stderr.strip()).strip()
            print(f"   Error:\n{output}")
            return False, output

    except FileNotFoundError:
        error_msg = f"Command not found: {command[0]}"
        print(f"[FAILED] {description} failed - {error_msg}")
        print(f"   Install with: pip install {command[0]}")
        return False, error_msg
    except Exception as e:
        error_msg = f"Unexpected error: {str(e)}"
        print(f"[FAILED] {description} failed - {error_msg}")
        return False, error_msg


def main():
    """Main function that runs all linting and testing commands."""
    print("Starting Control of Mobile Robots linting and testing...")
    print("=" * 60)

    # Define commands to run
    commands = [
        (["isort", "."], "Sorting imports with isort"),
        (["black", "."], "Formatting Python code with black"),
        (
            ["flake8", ".", "--ignore=E501,W503"],
            "Linting Python code with flake8",
        ),
        (["pytest"], "Running Python tests with pytest"),
    ]

    # Track results
    results = []
    all_passed = True

    # Run each command
    for command, description in commands:
        success, _ = run_command(command, description)
        results.append((description, success))
        if not success:
            all_passed = False
            # Continue with other checks instead of stopping

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY:")

    for description, success in results:
        status = "[PASSED]" if success else "[FAILED]"
        print(f"   {status}: {description}")

    if all_passed:
        print("\nAll checks passed! Your code is ready to go!")
        sys.exit(0)
    else:
        print("\nSome checks failed. Please review the output above.")
        print("\nTo fix formatting issues automatically, run:")
        print('   pip install -e ".[testing]"')
        print("   isort .")
        print("   black .")
        print("   flake8 . --ignore=E501,W503")
        print("   pytest")
        sys.exit(1)


if __name__ == "__main__":
    main()
