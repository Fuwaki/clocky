import * as wasm from "clocky"

wasm.default().then(() => {
    console.log("WASM loaded");
    wasm.init();
    wasm.start();
  });