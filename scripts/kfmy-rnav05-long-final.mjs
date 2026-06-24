import { runKfmyRnav05LongFinal } from "../streamdeck-plugin/src/scenarios/kfmy-rnav05-long-final.js";

const result = await runKfmyRnav05LongFinal();
console.log("Placed aircraft on KFMY RNAV 05 long final:", result);
