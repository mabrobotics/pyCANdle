#!/usr/bin/env bash
python3 setup.py sdist
pipx run cibuildwheel --platform linux --archs x86_64
python3 -m twine upload dist/* wheelhouse/*