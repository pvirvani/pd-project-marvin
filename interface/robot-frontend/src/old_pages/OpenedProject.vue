<template>
  <!-- <q-page class="flex flex-center"> -->

  <!-- <div class="q-pa-md q-gutter-y-md column items-start flex flex-center"> -->
  <!-- <q-btn-group push>
      <q-btn push label="First" icon="timeline" />
      <q-btn push label="Second" icon="visibility" />
      <q-btn push label="Third" icon="update" />
    </q-btn-group> -->

  <!-- <q-btn-group push>
      <q-btn to="/demo" color="amber-2" glossy text-color="black" push label="Demo" icon="precision_manufacturing"
        size='50px' />
      <q-btn to="/actions" color="amber-4" glossy text-color="black" push label="Actions" icon="question_mark"
        size='50px' />
      <q-btn to="domain" color="amber-6" glossy text-color="black" push label="Domain" icon="domain" size='50px' />
      <q-btn to="/problems" color="amber-8" glossy text-color="black" push label="Problems" icon="rule" size='50px' />
    </q-btn-group>
  </div> -->
  <div class=" q-pa-md q-gutter-y-md column items-start flex">
    <br />

    <p>
      <b>Project1</b>
    </p>
    <br />
  </div>
  <div>
    <div class=" q-pa-md q-gutter-y-md column items-start flex"
         style="float: left">
      <q-btn to="/demo"
             color="amber-2"
             text-color="black"
             push
             label="Demonstration"
             icon="o_precision_manufacturing"
             size='40px'
             style="width:350px; font-size:30px" />

      <q-btn to="/actions"
             color="amber-4"
             text-color="black"
             push
             label="Actions"
             icon="o_question_mark"
             size='40px'
             style="width:350px; font-size:30px" />
      <q-btn to="domain"
             color="amber-6"
             text-color="black"
             push
             label="Domain"
             icon="o_domain"
             size='40px'
             style="width:350px; font-size:30px" />
      <q-btn to="/problems"
             color="amber-8"
             text-color="black"
             push
             label="Problems"
             icon="o_rule"
             size='40px'
             style="width:350px; font-size:30px" />
    </div>
    <div class=" q-pa-md q-gutter-y-md column items-start flex"
         style="float: auto;background-color:antiquewhite">

      <br />
      <br />
      <br />
      <br />
      <br />
      <br />
      <p style="font-family: 'Times New Roman', Times, serif; text-align:center; font-size:30px"> Start with
        Demonstrations </p>
      <br />
      <br />
      <br />
      <br />
      <br />
      <br />
      <br />


    </div>
  </div>



  <!-- </q-page> -->
</template>

<script>
import { defineComponent } from 'vue'

export default {
  defineComponent() {
    name: 'OpenedProject'
  }
}

</script>
