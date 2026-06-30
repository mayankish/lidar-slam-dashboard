# Builds the C++ bot simulator. Plain POSIX sockets only, no external
# deps, so this works on any Linux box (including WSL2) with g++ and make:
#
#   sudo apt install g++ make   # if not already present
#   make
#   ./simulate_bot
#
# See ../README.md for full usage.

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra
TARGET   := simulate_bot
SRC      := simulate_bot.cpp

.PHONY: all clean run

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
