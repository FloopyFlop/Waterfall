"""
Orchestra service entrypoint that wraps the vendored drone_api test_demo.
Keeps a single visible file under waterfall/ for the Orchestra service.
"""

from waterfall.drone_api.test_demo import main  # re-export as entrypoint

if __name__ == "__main__":
    main()
