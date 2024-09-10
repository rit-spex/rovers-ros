SHELL = /bin/bash

.PHONY: help repo-init bootstrap check refresh-lock test

.DEFAULT_GOAL = help

# load environment variables from .env
ifneq (,$(wildcard ./.env))
    include .env
    export
endif

help:  ## Display this help
	@awk 'BEGIN {FS = ":.*##"; printf "\nUsage:\n  make \033[36m<target>\033[0m\n\nTargets:\n"} /^[a-zA-Z0-9_-]+:.*?##/ { printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2 }' $(MAKEFILE_LIST)

repo-init:  ## Install pre-commit in repo
	pre-commit install

bootstrap:  ## Install/update required packages (plus dev tools)
	poetry install --extras "dev"

check:  ## Run pre-commit against all files
	pre-commit run --all-files

refresh-lock:  ## Refresh poetry lock file without update
	poetry lock --no-update

test:  ## Run tests
	@echo Tests not implemented!
