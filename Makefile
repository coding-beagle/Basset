# Basset — common development tasks.
#
# `make` or `make help` lists every target. Targets are documented inline with a
# `## ` comment; the help rule parses those, so a new target documents itself.

CARGO   ?= cargo
PROFILE ?= release
PKG     ?= basset-app
FILE    ?=

# The app is unusable in an unoptimised build (see the profile note in Cargo.toml), so
# `run` defaults to release. `PROFILE=dev make run` gets a faster compile instead.
ifeq ($(PROFILE),dev)
PROFILE_FLAG :=
else
PROFILE_FLAG := --profile $(PROFILE)
endif

# Passed to `cargo run` after `--`; empty unless FILE was given.
RUN_ARGS := $(if $(FILE),-- $(FILE),)

.DEFAULT_GOAL := help
.PHONY: help run build build-dev test test-crate check fmt fmt-check clippy lint doc doc-open \
        audit outdated tree update clean distclean ci

help: ## List the available targets
	@echo "Basset — make targets"
	@echo
	@grep -hE '^[a-zA-Z_-]+:.*?## ' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}'
	@echo
	@echo "Variables (make VAR=value target)"
	@echo "  PROFILE=$(PROFILE)      cargo profile for build/run: release | dev"
	@echo "  PKG=$(PKG)     package for run/build/test-crate"
	@echo "  FILE=           .bass document to open with 'make run'"
	@echo
	@echo "Examples"
	@echo "  make run FILE=model.bass     open a document in the app"
	@echo "  make run PROFILE=dev        quick compile, slow viewport"
	@echo "  make test-crate PKG=basset-kernel"

run: ## Build and launch the desktop app (needs a Vulkan-capable GPU)
	$(CARGO) run $(PROFILE_FLAG) -p $(PKG) $(RUN_ARGS)

build: ## Build the whole workspace
	$(CARGO) build --workspace $(PROFILE_FLAG)

build-dev: ## Build the whole workspace with the dev profile
	$(CARGO) build --workspace

test: ## Run the workspace test suite
	$(CARGO) test --workspace --all-targets

test-crate: ## Run one crate's tests (PKG=basset-kernel)
	$(CARGO) test -p $(PKG) --all-targets

check: ## Type-check everything without producing binaries
	$(CARGO) check --workspace --all-targets

fmt: ## Format the source tree
	$(CARGO) fmt --all

fmt-check: ## Fail if anything is unformatted
	$(CARGO) fmt --all -- --check

clippy: ## Lint with clippy, warnings are errors
	$(CARGO) clippy --workspace --all-targets -- -D warnings

lint: fmt-check clippy ## Formatting check plus clippy

doc: ## Build the API documentation
	$(CARGO) doc --workspace --no-deps

doc-open: ## Build the API documentation and open it
	$(CARGO) doc --workspace --no-deps --open

audit: ## Check dependencies for advisories (needs cargo-audit)
	$(CARGO) audit

outdated: ## List dependencies with newer releases (needs cargo-outdated)
	$(CARGO) outdated --workspace --root-deps-only

tree: ## Show the dependency tree
	$(CARGO) tree --workspace

update: ## Update Cargo.lock within the declared version ranges
	$(CARGO) update

ci: lint test ## What CI runs: formatting, clippy, tests

clean: ## Remove build artefacts
	$(CARGO) clean

distclean: clean ## Remove build artefacts and Cargo.lock
	rm -f Cargo.lock
