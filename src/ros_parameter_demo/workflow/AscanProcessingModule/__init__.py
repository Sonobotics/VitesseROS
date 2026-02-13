import platform
import sys
import os
from importlib import import_module


def _get_arch():
    """Get the normalized architecture name."""
    machine = platform.machine().lower()

    # Normalize architecture names
    arch_map = {
        'x86_64': 'x86_64',
        'amd64': 'x86_64',
        'aarch64': 'arm64',
        'arm64': 'arm64',
    }

    return arch_map.get(machine, machine)


def _load_module():
    """Load the appropriate compiled binary for this architecture."""
    arch = _get_arch()
    module_dir = os.path.dirname(__file__)
    binary_path = os.path.join(module_dir, "objects", arch)

    if not os.path.exists(binary_path):
        raise ImportError(
            f"AscanProcessingModule: No compiled binary found for architecture '{arch}'. "
            f"Expected binary at: {binary_path}/_ascan.so\n"
        )

    # Add binary path to sys.path temporarily
    if binary_path not in sys.path:
        sys.path.insert(0, binary_path)

    try:
        # Import the compiled module
        _impl = import_module('_ascan')
        return _impl
    except ImportError as e:
        raise ImportError(
            f"AscanProcessingModule: Found binary directory for '{arch}' but failed to load: {e}"
        ) from e
    finally:
        # Clean up sys.path
        if binary_path in sys.path:
            sys.path.remove(binary_path)


# Load the implementation
_impl = _load_module()

# Export all public functions from the implementation
singularThicknessCalculation = _impl.singularThicknessCalculation

# Export metadata
__all__ = ['singularThicknessCalculation']
__version__ = '1.0.0'
