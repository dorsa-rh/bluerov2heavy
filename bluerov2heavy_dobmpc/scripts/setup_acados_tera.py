#!/usr/bin/env python3
"""
Setup script to fix Acados tera version compatibility issues.
Run this before generate_c_code.py to ensure the correct tera binary is available.
"""

import sys

def setup_tera():
    """
    Download and set up compatible tera version for Acados.
    This fixes issues where older Linux systems have incompatible libc.so.
    """
    try:
        from acados_template import get_tera
        print("Setting up compatible tera version (0.0.34)...")
        get_tera(tera_version='0.0.34', force_download=True)
        print("✓ Tera setup completed successfully!")
        return True
    except Exception as e:
        print(f"✗ Error setting up tera: {e}", file=sys.stderr)
        return False

if __name__ == '__main__':
    success = setup_tera()
    sys.exit(0 if success else 1)
