build:
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --no-warn-unused-cli

test:
	colcon test && colcon test-result

.PHONY: build test