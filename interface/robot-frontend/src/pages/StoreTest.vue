<template>
    <div>
       pdata is {{ pdata}} {{store.getparameter() }}
    </div>
</template>

<script setup>
import { action_parameter } from 'src/stores/action_parameter';
import { ref } from 'vue';
    let pdata =ref ([])
    const store = action_parameter()
    pdata = store.getparameter()
    store.setparameter('help')


</script>
