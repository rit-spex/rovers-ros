DC = docker compose

build:
	$(DC) build

build-no-cache:
	$(DC) build --no-cache

enter: build
	$(DC) up -d
	$(DC) exec ros2 /bin/bash --init-file init_file.sh

down:
	$(DC) down
