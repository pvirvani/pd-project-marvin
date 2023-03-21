<template>

  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="background-color:bisque">
    <p style="font-size:20px"><b>Project 1</b></p>
  </div>
  <br />
  <br />
  <!-- buttons -->
  <!-- <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="float: left; background-color:antiquewhite">
    <q-btn to="/demo"
           color="primary"
           push
           label="Demonstration"
           icon="o_precision_manufacturing"
           size='40px'
           style="width:350px; font-size:30px" />

    <q-btn to="/actions"
           color="primary"
           push
           label="Actions"
           icon="o_question_mark"
           size='40px'
           style="width:350px; font-size:30px" />
    <q-btn to="domain"
           color="primary"
           push
           label="Domain"
           icon="o_domain"
           size='40px'
           style="width:350px; font-size:30px" />
    <q-btn to="/problems"
           color="primary"
           push
           label="Problems"
           icon="o_rule"
           size='40px'
           style="width:350px; font-size:30px" />
  </div> -->
  <div class=" q-pa-md q-gutter-y-md column items-start flex"
       style="float: left; background-color:antiquewhite">
    <q-btn to="/demo"
           color="primary"
           push
           label="Demonstration"
           icon="o_precision_manufacturing"
           size='40px'
           style="width:350px; font-size:30px" />

    <q-btn to="/actions"
           color="primary"
           push
           label="Actions"
           icon="o_question_mark"
           size='40px'
           style="width:350px; font-size:30px" />
    <q-btn to="/problems"
           color="primary"
           push
           label="Problems"
           icon="o_rule"
           size='40px'
           style="width:350px; font-size:30px" />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <br />
    <q-btn to="domain"
           color="primary"
           push
           label="Domain"
           icon="o_domain"
           size='40px'
           style="width:350px; font-size:30px" />
  </div>


  <!-- <div class=" q-pa-md q-gutter-y-md column items-start flex" style="float: auto;background-color:antiquewhite" > -->
  <div class="q-pa-md row items-start q-gutter-md">
    <q-card class="my-card bg-secondary text-white">
      <q-scroll-area style="height: 650px; max-width: 600px;">
        <q-card-section>
          <div class="text-h6"
               style="text-align:center">
            <u>Requirements</u>
          </div>
          <!-- <div class="text-subtitle2">by John Doe</div> -->
        </q-card-section>

        <q-card-section>
          <pre>{{ requirements }}</pre>
        </q-card-section>


        <q-card-section>
          <div class="text-h6"
               style="text-align:center">
            <u>Types</u>
          </div>
          <!-- <div class="text-subtitle2">by John Doe</div> -->
        </q-card-section>
        <q-card-section>
          <pre>{{ types }}</pre>
        </q-card-section>


        <q-separator dark />

        <q-card-actions align="left">
          <!-- <q-btn class="glossy" aligh="center" icon="file_upload" rounded color="deep-orange" label="Export Domain" /> -->
          <!-- <q-btn flat>Action 2</q-btn> -->
          <q-btn :loading="progress[0].loading"
                 :percentage="progress[0].percentage"
                 color="primary"
                 @click="startComputing(0)"
                 style="width: 150px"
                 icon="file_upload"
                 aligh="center">
            Export Domain
            <template v-slot:loading>
              <q-spinner-gears class="on-left" />
              Exporting...
            </template>
          </q-btn>
        </q-card-actions>
      </q-scroll-area>
    </q-card>


  </div>
  <!-- </div> -->
</template>

<script>
import { defineComponent, onBeforeUnmount, ref } from 'vue'


export default {
  defineComponent() {
    name: 'OnDomain'
  },
  setup() {

    const progress = ref([
      { loading: false, percentage: 0 },
      { loading: false, percentage: 0 },
      { loading: false, percentage: 0 }
    ])

    const intervals = [null, null, null]

    function startComputing(id) {
      progress.value[id].loading = true
      progress.value[id].percentage = 0

      intervals[id] = setInterval(() => {
        progress.value[id].percentage += Math.floor(Math.random() * 8 + 10)
        if (progress.value[id].percentage >= 100) {
          clearInterval(intervals[id])
          progress.value[id].loading = false
        }
      }, 700)
    }

    onBeforeUnmount(() => {
      intervals.forEach(val => {
        clearInterval(val)
      })
    })


    return {
      requirements: `(:requirements
 :strips                   ;       add delete effects.
 :typing                   ;       add types
 :hierarchy                ;       add tasks - subtasks ... etc.
 :method-preconditions     ;       to the methods.
 :negative-preconditions   ;       add negative preconditions.
 ) `,
      types: `(:types
 Point_Air - Location           ;   Point at Air
 Point Workspace - Location     ;   Workspace Point
 Location - Object
 Block_2x2 - Block              ;   Block 2x2
 Block_2x4 - Block              ;   Block 2x4
 Block_2x6 - Block              ;   Block 2x6
 Block - Object
 )`,

      progress,
      startComputing,
    }//return
  }//setup
}//export
</script>

<style lang="sass" scoped>
.my-card
  width: 100%
  max-width: 600px
</style>
