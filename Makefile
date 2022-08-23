MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

CONTAINERNAME?=vicon-receiver
NETWORK?=host
ENV?=
PORT?=-p 801:801

all: build

build:
	$(BAKE) $(CONTAINERNAME) 

local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push $(CONTAINERNAME)

run: build
	docker run -it --rm --net=$(NETWORK) $(PORT) $(ENV) uobflightlabstarling/$(CONTAINERNAME):latest

run_bash: build
	docker run -it --rm --net=$(NETWORK) $(PORT) $(ENV) uobflightlabstarling/$(CONTAINERNAME):latest bash

.PHONY: all build local-build-push run run_bash run_gimbal
