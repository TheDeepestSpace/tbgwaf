// Single source of truth for the map choices offered on the web splash.
// Ids are understood by src/main.cpp (Module.tbgwafMap); labels mirror the
// buttons on the game's own Map Select screen (src/ui/Menu.cpp).
export const MAPS = Object.freeze([
  Object.freeze({ id: "urban", label: "Urban" }),
  Object.freeze({ id: "urban-x4", label: "Urban x4" }),
  Object.freeze({ id: "hills", label: "Hills" }),
]);
export const DEFAULT_MAP = "urban";
// Matches the web default seed in src/main.cpp.
export const DEFAULT_SEED = 7;

export function parseSeed(text) {
  const n = Number(String(text ?? "").trim());
  return Number.isInteger(n) && n >= 0 && n <= 0xffffffff ? n : DEFAULT_SEED;
}
export function randomSeed() {
  return Math.floor(Math.random() * 0x100000000);
}
export function matchOptions(params) {
  const map = MAPS.some((m) => m.id === params.get("map")) ? params.get("map") : DEFAULT_MAP;
  return { map, seed: parseSeed(params.get("seed")) };
}
