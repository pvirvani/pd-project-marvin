<template>
  <div>
    <div v-if="connected == true">ros is connected
      <!-- {{ disconnect() }} -->
        <br>
        lego data from perception module: {{ldata}}
        <br>
        store data complete: {{ astore.getparameter() }}
        <br>
        last pushed data in the store: {{ store_data[store_data.length - 1] }}

        <div v-if="ldata.length >= 1 && ldata != store_data[store_data.length - 1]">
        {{ showNotif() }}
        <!-- <q-btn label="show notice" color="primary" @click="showNotif()"/> -->
        </div>
      </div>
      <div class="q-pa-md">
        <q-card class="my-card">
          <q-video style="height: 700px;"
                   src="http://localhost:3000/" />
          <q-card-section>
            <div class="text-h6">GUI PacBot</div>
          </q-card-section>
        </q-card>
      </div>
    <!-- </div> -->
    <div v-if="connected == false">ros is not connected
    </div>
  </div>
</template>
<script setup>
import "roslib/build/roslib";
import { ref } from "vue";
import { useQuasar } from "quasar";
import { action_parameter } from "src/stores/action_parameter";
// // -----------------------------
// // Connection for Jetson Nano //
// // -----------------------------
// const ws_address = 'ws://192.168.125.50:9090';

// // -----------------
// // Local Connection With PC
// // -----------------
const ws_address = 'ws://localhost:9090'
// const ws_address = 'ws://129.88.67.254:9090';
// const ws_address = 'ws://192.168.125.10:9090';
// const ws_address = 'ws://193.54.184.6:9090';
let pdata = ref("")
let vdata = ref("")
let ldata = ref("")
const astore = action_parameter()
const $q = useQuasar()
let rosEvent = ref(false)
let position_validator = ref(false)
let lego_data = ref([])
let connected = ref(false);
let store_data = ref([])
const ros = new window.ROSLIB.Ros({
  url: ws_address,
});

ros.on("connection", () => {
  connected.value = true;
  console.log("Connected!");
});
ros.on("error", (error) => {
  console.log("Error connecting to websocket server: ", error);
});
ros.on("close", () => {
  connected.value = false;
  console.log("Connection to websocket server closed.");
});
const disconnect = () => {
  // console.log("Connection to websocket server disconnected.");
  ros.close();
};
// ###################################
// 
//      Subscribing to lego_map
// 
// ###################################
// const legomap = new window.ROSLIB.Topic({
//   ros: ros,
//   name: '/lego_map',
//   messageType: 'std_msgs/String'
// });

// legomap.subscribe(function (message) {
//   console.log('Receiving data on ' + legomap.name + ':' + message.data);
//   pdata.value = message.data
//   legomap.unsubscribe();
// });

// ###################################
// 
//      Subscribing to vision_dict
// 
// ###################################
// const visiondict = new window.ROSLIB.Topic({
//   ros: ros,
//   name: '/vision_dict',
//   messageType: 'std_msgs/String'
// });

// visiondict.subscribe(function (message) {
//   console.log('Receiving datafrom Lego_Map.json file on ' + visiondict.name + ':' + message.data);
//   vdata.value = message.data
//   visiondict.unsubscribe();
// });

// ###################################
// 
//      Subscribing to lego_data_pub topic
// 
// ###################################
const legoros = new window.ROSLIB.Topic({
  ros: ros,
  name: '/lego_data',
  messageType: 'std_msgs/String'
});

legoros.subscribe(function (message) {
  console.log('Receiving datafrom position publisher on ' + legoros.name + ':' + message.data);
  ldata.value = message.data
  // legoros.unsubscribe();
});
function call_lego(){
legoros.subscribe(function (message) {
  console.log('Receiving datafrom position publisher on ' + legoros.name + ':' + message.data);
  ldata.value = message.data
  legoros.unsubscribe();
});
}
// legoros.subscribe(function (message) {
//   console.log('Receiving datafrom position publisher on ' + legoros.name + ':' + message.data);
//   ldata.value = message.data
//   // legoros.unsubscribe();
// });





// function subscriber_local(){
//   legoros.subscribe(function (message) {
//   console.log('Receiving datafrom position publisher on ' + legoros.name + ':' + message.data);
//   ldata.value = message.data
//   legoros.unsubscribe();
// });
// }

store_data = astore.getparameter()

function showNotif() {
        $q.notify({
          timeout: 0,
          progress: true,
          message: 'Lego block has been placed at '+ldata.value+' . Do you want to validate the parameters?',
          color: 'none',
          multiLine: true,
          avatar: 'https://cdn.quasar.dev/img/boy-avatar.png',
          actions: [
            { label: 'Cancel', color: 'red', handler: () => { /* ... */ } },
            { label: 'Validate', color: 'primary', handler: () => {
              // lego_data.value.push(ldata.value)
              astore.setparameter(ldata.value)
              ldata.value = {}
            } }
            
          ]
        })
      }

</script>
