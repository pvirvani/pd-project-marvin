import { defineStore } from "pinia";

export const global_pid = defineStore("global_pid", {
  state: () => ({
    global_pid: "",
    global_pname:""
  }),
  getters: {
    // doubleCount: (state) => state.counter * 2,
    // getparameter: (state) => state.aparameter
  },
  actions: {
    setgid(gid,gname) {
    //   this.aparameter = pdata
      this.global_pid = gid
      this.global_pname= gname
    },
    getgid(){
      return this.global_pid
    },
    getgname() {
      return this.global_pname
    }
    // removegid(){
    //     this.global_pid.pop()
    // }
  },
});
