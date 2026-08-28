DC = docker compose

build-dc:
	$(DC) build

enter: build-dc
	$(DC) up -d
	$(DC) exec ros2 /bin/bash --init-file init_file.sh

down:
	$(DC) down
