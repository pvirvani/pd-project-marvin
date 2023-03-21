import { createApp } from "vue";
import App from './App.vue'
import router from "./router";
import { FontAwesomeIcon } from "@fortawesome/vue-fontawesome";
import { library } from "@fortawesome/fontawesome-svg-core";
import { faUser, faRobot, faArrowLeft, faArrowRight, faSave, faArrowDown, faX, faAnglesRight } from "@fortawesome/free-solid-svg-icons";
import mitt from "mitt";

library.add(faUser, faRobot, faArrowLeft, faArrowRight, faSave, faArrowDown, faX, faAnglesRight);

const emitter = mitt();

const app = createApp(App);

app.config.globalProperties.emitter = emitter;
app.use(router)
app.component("font-awesome-icon", FontAwesomeIcon)
app.mount("#app");