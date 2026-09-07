#!/usr/bin/env python3
"""Run one A6 core request in the frozen research Python environment."""

from a6_pilot.worker import main


if __name__ == '__main__':
    raise SystemExit(main())
