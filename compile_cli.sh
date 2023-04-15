rm -rf build
cmake -GNinja -Bbuild -DSDK_CLEAR_ENV=TRUE .
cmake --build build