import { defineStore } from "pinia";

export const action_parameter = defineStore("action_parameter", {
  state: () => ({
    aparameter: [],
  }),
  getters: {
    // doubleCount: (state) => state.counter * 2,
    // getparameter: (state) => state.aparameter
  },
  actions: {
    setparameter(pdata) {
    //   this.aparameter = pdata
      this.aparameter.push(pdata)
    },
    getparameter(){
        return this.aparameter
    },
    removeparameter(){
        this.aparameter.pop()
    }
  },
});
