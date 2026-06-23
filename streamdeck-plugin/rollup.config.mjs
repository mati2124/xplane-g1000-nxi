import resolve from "@rollup/plugin-node-resolve";

export default {
  input: "src/plugin.js",
  output: {
    file: "com.andy.xplane-nxi.sdPlugin/bin/plugin.js",
    format: "esm",
  },
  plugins: [resolve({ preferBuiltins: true })],
  external: ["node:child_process"],
};
