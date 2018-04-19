# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SPHINXPROJ    = DiscreteEventSimulationinSwarmRobotics
SOURCEDIR     = .
BUILDDIR      = _build

.PHONY: all
all: html

api/:
	mkdir -p api

api/swarmdes.rst: ../swarmdes.py | api/
	sphinx-apidoc -o api ..

.PHONY: help Makefile html

html: Makefile api/swarmdes.rst
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
