import { ExtensionContext } from "@foxglove/extension";
import { initArgoSailboatPanel } from "./ArgoSailboatPanel";

export function activate(extensionContext: ExtensionContext) {
  extensionContext.registerPanel({
    name: "argo-sailboat-panel",
    initPanel: initArgoSailboatPanel,
  });
}

