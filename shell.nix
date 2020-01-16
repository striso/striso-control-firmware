with import <nixpkgs> {
  crossSystem = {
    config = "arm-none-eabi";
    libc = "newlib";
  };
};

mkShell {
  buildInputs = [];
}
