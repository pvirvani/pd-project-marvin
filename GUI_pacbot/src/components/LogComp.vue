<!-- 
    PlanLego.vue 
    ---
    @author: Lukas Loiodice.
    LIG/Marvin France, 2022.
    ---
    @modified by: Belal HMEDAN.
    LIG/Marvin France, 2022.
    ---
    Plan Template. 
-->
<template>
    <li class="Log"  :id="id">
        <!-- This part to Parse Log Message -->
        <div id="logsText">
            <!-- Cyan Color for Date/Time-->
            <p :style="style"> [{{ type }}]</p>
            <p :style="style"> {{ message }}</p>
            <p :style="`color: #2bfafa; text-align: end;`"> [{{ hours }}] </p>
        </div>
    </li>
</template>

<script lang="js">

export default {
    name: "LogComp",
    props: {
        time_stamp: {
            type: String,
            required: true
        },
        // Event Number (ID)
        name: {
            type: Number,
            required: true
        },
        // Event Message
        message: {
            type: String,
            required: true
        },
        // Event Color: {White: Normal, Orange: Warning, Red: Error}
        color: {
            type: String,
            required: true
        }
    },
    computed: {
        // Change Event Color Automatically.
        style() {
            return 'color: ' + this.color; 
        },
        history(){
            let utcStr = new Date();
            const timeDiff = 2* (60*60*1000); // Two Hours GMT + 2
            utcStr.setTime(utcStr.getTime() + timeDiff);
            return utcStr.toUTCString();
        },
        type(){
            /* 
            * Event Type, Normal is not considered! 
            */
            let col = "unknown";
            if (this.color === "red"){
                col = "error"
            }else if (this.color === "orange"){
                col = "warning"
            }
            else if (this.color === "white"){
                col = "info"
            }
            return col;
        },
        id(){
            return "event"+this.name
        }
    },
    methods: {
        manageEvents(event){
            if (event.type == "cancelRemove"){
                this.$emit('cancelRemove', {name: event.value})
                /* 
                * This Part Needs Fix (Line through not working)
                */
                // let text =
                // document.getElementById("event"+this.name).children[0].children;
                // for (let i = 0; i < text.length; i++){
                // text[i].style.textDecoration = "line-through";
                // }
            }
        }
    },
    // LifeCycle Hook
    beforeCreate(){
        let utcStr = new Date();
        const timeDiff = 2* (60*60*1000); // Two Hours GMT + 2
        utcStr.setTime(utcStr.getTime() + timeDiff);
        this.hours = utcStr.toUTCString();
    }
}
</script>

<style>
    @import '../style/css/LogComp.css';
</style>