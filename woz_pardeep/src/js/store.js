import { reactive } from "vue";

export const store = reactive({
  pattern: null,
  lego_map: null,
  model: null,
  pick_enabled: false,
  place_enabled: false,
  rotated: false,
  picked_lego: null,
});
