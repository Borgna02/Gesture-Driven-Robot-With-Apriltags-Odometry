const route = (event) => {
    event = event || window.event;
    event.preventDefault();

    window.history.pushState({}, "", event.target.href);
    handleLocation();
};

const routes = {
    $#ROUTES#$
};

const handleLocation = async () => {
    const path = window.location.pathname;
    const route = routes[path] || routes[404];

    notifier.send(JSON.stringify(route));
};

window.onpopstate = handleLocation;
window.route = route;

handleLocation();
