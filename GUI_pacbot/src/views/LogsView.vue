<template>
    <div id="logs" class="param">
        <ul id="logUl">
        <LogComp
            v-for = "log in store.logs"
            :time_stamp="log.time_stamp"
            :name = "log.name"
            :message = "log.message"
            :color = "log.color"
            :key = "log.name+log.message"
            :ref="`event-ref-̀`+log.name"
            @cancelRemove ="cancelRemove"
        />
        </ul>
    </div>

</template>

<script lang="js">
import { store } from "../js/store.js";
import LogComp from "../components/LogComp.vue";

export default {
    name: "LogsView",
    data(){
        return{
            store,
        }
    },
    created (){
            this.emitter.on('save-logs', () => {
                this.saveLogs();
            });
            this.emitter.off('save-logs', () => {
                this.saveLogs();
            });
    },
    methods: {
        cancelRemove(event){
            let step = "+sl"+event.name;
            console.log(step);
        },
        /****************************************
        * Convert a 2D array into a CSV string
        ****************************************
        * 
        * @param {*} data 
        */
        arrayToCsv(data) {
        /*
        return data.map(row =>
            row
            .map(String)  // convert every value to String
            .map(v => v.replaceAll('"', '""'))  // escape double colons
            .map(v => `"${v}"`)  // quote it
            .join(',')  // comma-separated
        ).join('\r\n');  // rows starting on new lines
        */
        const array = [Object.keys(data[0])].concat(data)

            return array.map(it => {
                return Object.values(it).toString()
            }).join('\n');
        },
        /******************************************
         * 
         * @param {*} content 
         * @param {*} filename 
         * @param {*} contentType 
         * downloadBlob(csv, 'export.csv', 'text/csv;charset=utf-8;')
         */
        downloadBlob(content, filename, contentType) {
        // Create a blob
        var blob = new Blob([content], { type: contentType });
        var url = URL.createObjectURL(blob);

        // Create a link to download it
        var pom = document.createElement('a');
        pom.href = url;
        pom.setAttribute('download', filename);
        pom.click();
        },
        saveLogs() { 
            const myLogs = this.arrayToCsv(store.logs);
            this.downloadBlob(myLogs, './gui_logs.csv', 'text/csv;charset=utf-8;');
            console.log("Logs has been saved");
        },
    },
    components: { LogComp }
}
</script>

<style>
    @import '../style/css/LogsView.css';
</style>