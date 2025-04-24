# Updating Clangd for LSP

By default, clangd doesn't know where to search for all the include statements.
You can help it out by providing the esp-idf library include paths. Run:

```
xtensa-esp32s3-elf-gcc -E -Wp,-v -xc /dev/null 2>&1 | sed -n '/#include <...> search starts here:/,/End of search list./p'
```

You will get an output like:

```
#include <...> search starts here:
 /nix/store/rzl126vpghy80qxq6jk35yi47ilqhvsq-xtensa-esp-elf-esp-idf-v5.4/xtensa-esp-elf/bin/../lib/gcc/xtensa-esp-elf/14.2.0/include
 /nix/store/rzl126vpghy80qxq6jk35yi47ilqhvsq-xtensa-esp-elf-esp-idf-v5.4/xtensa-esp-elf/bin/../lib/gcc/xtensa-esp-elf/14.2.0/include-fixed
 /nix/store/rzl126vpghy80qxq6jk35yi47ilqhvsq-xtensa-esp-elf-esp-idf-v5.4/xtensa-esp-elf/bin/../lib/gcc/xtensa-esp-elf/14.2.0/../../../../xtensa-esp-elf/include
End of search list.
```

Now, take the paths, prefix them with `-I` and add it to the `.clangd` CompileFlags.Add array.

Furthermore, its best you compile the firmware at least once to generate a valid `compile_commands.json` in the `build/` directory.

> [!NOTE]
> If you are not targeting an ESP32-S3, change the `xtensa-esp**-elf-gcc` command to match your target architecture.

# Editor tasks

If you are using zeditor, you can press `ALT` + `SHIFT` + `T` to see a list of tasks. Currently there is compile, upload, and serial monitor. See `.zed/tasks.json`.
