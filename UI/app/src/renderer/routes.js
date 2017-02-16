export default [
    {
        path: '/',
        name: 'home',
        component: require('components/Home')
    },
    {
        path: '*',
        redirect: '/'
    }
]
