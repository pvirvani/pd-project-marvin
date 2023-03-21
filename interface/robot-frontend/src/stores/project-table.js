import { defineStore } from "pinia";

export let projectStore = defineStore("projectId", {
  state: () => ({
    projectId: 0,
    projectName: [],
    selectedItems: [],

    // projectProblems: 10,
    // projectActions: 3,
  }),
  getters: {
    // doubleCount: (state) => state.counter * 2,
    projectDescription: (state) =>
      `${state.projectName} ${state.projectProblems} ${state.projectActions}`,
  },
  actions: {
    //     increament() {
    //       this.projectId++;
    //     },
    addProject(pname) {
      this.projectName.push({
        id: this.projectId++,
        pname,
        projectProblems: 10,
        projectActions: 3,
      });
    },

    // deleteProject(indx) {
    //   if (indx.length === 0) {
    //     return
    //   } else {
    //     // for (let i = 0; i < selectedItems.length; i++) {
    //     //   this.projecName = this.projecName.filter((object) => {
    //     //     return object.id !== selectedItems[i].id;
    //     //   });
    //     // }
    //     for (let i = 0; i < indx.length; i++) {
    //       this.projecName.splice(this.projectName.selectedItems[i].id, 1);
    //     }
    //   }
    // },

    deleteProject(pID) {
      this.projectName = this.projectName.filter((object) => {
        return object.id !== pID;
      });
    },

    editProject(pID, item) {
      this.projectName.filter((object) => {
        if (object.id === pID) {
          object.pname = item;
        }
      });
    },
  }, //action
}); //export
