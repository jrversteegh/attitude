# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys
from pathlib import Path

project = "Propeller Performance"
copyright = "2022, Damen Research, Development and Innovation"


from proper import *
from proper import __author__, __version__

release = __version__
author = __author__
script_dir = Path(os.path.dirname(os.path.realpath(__file__)))

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "sphinx.ext.graphviz",
    "breathe",
]

# Breathe configuration
breathe_projects = {"properxx": f"{script_dir/'build'/'doxygen'/'xml'}"}
breathe_default_project = "properxx"

exclude_patterns = ["build", "Thumbs.db", ".DS_Store"]


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_static_path = ["_static"]
html_theme = "sphinx_rtd_theme"
# html_theme = 'alabaster'
# html_theme = "basic-bootstrap"
# templates_path = ["_templates"]
# html_theme_path = ["_themes"]
html_theme_options = {
    "collapse_navigation": False,
}

root_doc = "index"

autosummary_generate = True

autodoc_default_options = {
    "members": True,
    "show-inheritance": True,
    "inherited-members": True,
    "no-special-members": True,
}
