
# Variables to hold directory paths
CJSON_DIR 	:= ./cJSON
SRC_DIR  	:= ./src
BIN_DIR 	:= ./bin

# Compiler and compiler flags
CC 	   	:= gcc
CFLAGS 	:= -I$(CJSON_DIR) -I$(SRC_DIR) -g
LFLAGS 	:=  -DASTCLCK -DLVALCLCK -DAST_HC_DBG

# Source files
SOURCES := 	$(CJSON_DIR)/cJSON.c \
           	$(SRC_DIR)/parser3.c \
           	$(SRC_DIR)/min_heap.c \
		   	$(SRC_DIR)/obstacle_marker.c \
        	$(SRC_DIR)/hom_classes.c \
           	$(SRC_DIR)/astar_homotopies.c

# Files for stand-alone boundary optimizations program				
BO_FILES := $(SRC_DIR)/ast_hom_bo.c

# Target to build the project
bo_hom_test: $(SOURCES) $(BO_FILES)
	$(CC) $(CFLAGS) $(LFLAGS) -DBO_LATEX -DBO_UT -DBO_LOG $^ -o $(BIN_DIR)/bo_hom_test

build_astar_ut: $(SOURCES)
	$(CC) $(CFLAGS) $(LFLAGS) -DAST_UT $^ -o $(BIN_DIR)/astar_ut

# Phony target to avoid conflicts with potential files named 'clean'
.PHONY: clean

# Clean up command
clean:
	rm -f $(BIN_DIR)/bo_hom_test
	rm -f $(BIN_DIR)/astar_ut
	
	