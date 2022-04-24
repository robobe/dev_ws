const app = Vue.createApp({
    data() { 
            return {
                title: "hello vue",
                counter: 0
            }
    },
    methods: {
        reset(){
            this.counter = 0
        },
        preset(value){
            this.counter = value
        }
    }
})
vm = app.mount('#app')
setTimeout(function(){
    vm.counter = 20;
}, 3000)