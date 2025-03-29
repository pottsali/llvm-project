# Notes for Mups16

## Building required binaries

```
cd ~/git/llvm-project/build
cmake --build . -j8 --target llvm-objdump llc

```

## Building test program

```
cd ~/git/llvm-project/build

# building the test program to IL
$ ./bin/clang -target mups16 -c ~/git/cpu/cpp/target/hello_world/main.c -emit-llvm

# compiling the IL to assembly
$ ./bin/llc -O0 -march=mups16 -relocation-model=static -filetype=asm main.bc -o main.s && cat main.s

# compiling the IL directly to ELF
$ ./bin/llc -O0 -march=mups16 -relocation-model=static -filetype=obj main.bc -o main.o

```
