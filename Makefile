
# Variables to hold directory paths
CJSON_DIR := ./cJSON
SRC_DIR := ./src
ASTAR_HOM_DIR := $(SRC_DIR)/astar_homotopies
EXEC_DIR := ./executables

# Compiler and compiler flags
CC := gcc
CFLAGS := -I$(CJSON_DIR) -I$(SRC_DIR) -I$(ASTAR_HOM_DIR) -g

# Source files
SOURCES := $(CJSON_DIR)/cJSON.c \
           $(SRC_DIR)/parser3.c \
           $(SRC_DIR)/min_heap.c

# Files for astar homotopy program
ASTAR_HOM_FILES :=	$(ASTAR_HOM_DIR)/obstacle_marker.c \
           			$(ASTAR_HOM_DIR)/hom_classes.c \
           			$(ASTAR_HOM_DIR)/astar_homotopies.c \
           			$(ASTAR_HOM_DIR)/ast_hom_bo.c

# Target to build the project
build_astar_homotopies: $(SOURCES) $(ASTAR_HOM_FILES)
	$(CC) $(CFLAGS) $^ -o $(EXEC_DIR)/bo_hom_test

# Phony target to avoid conflicts with potential files named 'clean'
.PHONY: clean

# Clean up command
clean:
	rm -f $(EXEC_DIR)/bo_hom_test
