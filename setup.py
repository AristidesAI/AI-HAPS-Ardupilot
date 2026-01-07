from setuptools import setup, find_packages

setup(
    name="hapsglider",
    version="0.1.0",
    description="HAPS Glider - ArduPilot AI Agent Integration for Stratospheric Operations",
    author="HAPS Development Team",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    python_requires=">=3.10",
    install_requires=[
        "pymavlink>=2.4.41",
        "dronekit>=2.9.2",
        "numpy>=1.24.0",
        "pyyaml>=6.0",
        "aiohttp>=3.8.0",
        "structlog>=23.1.0",
        "pyproj>=3.5.0",
        "astral>=3.2",
    ],
    extras_require={
        "dev": [
            "pytest>=7.3.0",
            "pytest-asyncio>=0.21.0",
        ],
        "ml": [
            "torch>=2.0.0",
        ],
    },
    entry_points={
        "console_scripts": [
            "hapsglider=main:main",
        ],
    },
)
