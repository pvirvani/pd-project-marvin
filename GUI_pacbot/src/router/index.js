import { createWebHistory, createRouter } from "vue-router";
import PageNotFound from '../views/PageNotFound.vue'

import SubPageForRouting from "../views/SubView.vue";
import LogsView from "../views/LogsView.vue";
import ControlsView from "../views/ControlsView.vue";
  
const routes = [
  {
    path: "/",
    component: SubPageForRouting,
    children: [
      {
        path: "",
        alias: "controls",
        name:"controls",
        component: ControlsView
      },
      {
        path: "logs",
        name:"logs",
        component: LogsView,
        props: null
      }
    ]
    },
    {
        path: '/:catchAll(.*)*',
        name: "PageNotFound",
        component: PageNotFound,
    },
];

const router = createRouter({
    history: createWebHistory(),
    routes
})

export default router