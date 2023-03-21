const routes = [
  // {
  //   path: "/",
  //   component: () => import("layouts/MainLayout.vue"),
  //   children: [{ path: "", component: () => import("pages/IndexPage.vue") }],
  // },
  // {
  //   path: "/phome",
  //   component: () => import("layouts/MainLayout.vue"),
  //   children: [{ path: "", component: () => import("pages/ProjectHome.vue") }],
  // },
  {
    path: "/",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/ProjectHomeNew.vue") },
    ],
  },
  {
    path: "/nhome",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/NewHomeTest.vue") }],
  },
  {
    path: "/nhomev1",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/NewHomeTest_v1.vue") },
    ],
  },
  {
    path: "/nhomev11",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/NewHomeTest_v11.vue") },
    ],
  },
  {
    path: "/nhomev2",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/NewHomeTest_v2.vue") },
    ],
  },
  {
    path: "/nhomev3",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/NewHomeTest_v3.vue") },
    ],
  },
  {
    path: "/tvue",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/TestVue.vue") }],
  },
  {
    path: "/tvf",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/TestVueFlask.vue") }],
  },
  {
    path: "/tvfa",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/TestVueFlaskFAlert.vue") },
    ],
  },
  {
    path: "/ping",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/PingP.vue") }],
  },
  {
    path: "/books",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/BooksB.vue") }],
  },
  {
    path: "/pflask",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/ProjectsP.vue") }],
  },
  {
    path: "/trial",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/THome.vue") }],
  },
  {
    path: "/newproject",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/CreateNewProject.vue") },
    ],
  },
  {
    path: "/openproject",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/ProjectsList.vue") }],
  },
  {
    path: "/openedproject",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/OpenedProject.vue") },
    ],
  },
  {
    path: "/demo",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("pages/OnDemonstration.vue") },
    ],
  },
  {
    path: "/actions",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/OnActions.vue") }],
  },
  {
    path: "/domain",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("pages/OnDomain.vue") }],
  },
  {
    path: "/problems",
    component: () => import("layouts/MainLayout.vue"),
    children: [{ path: "", component: () => import("src/old_pages/OnProblems.vue") }],
  },
  {
    path: "/openproblems",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("src/old_pages/OnOpenProblems.vue") },
    ],
  },
  {
    path: "/problemvalidation",
    component: () => import("layouts/MainLayout.vue"),
    children: [
      { path: "", component: () => import("src/old_pages/OnProblemValidation.vue") },
    ],
  },

  // Always leave this as last one,
  // but you can also remove it
  {
    path: "/:catchAll(.*)*",
    component: () => import("pages/ErrorNotFound.vue"),
  },
];

export default routes;
