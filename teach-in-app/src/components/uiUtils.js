export function toggleVisibility(showElementIds = [], hideElementIds = []) {
    showElementIds.forEach(id => {
        const element = document.getElementById(id);
        element.classList.remove('hidden');
        element.offsetHeight; 
    });
    hideElementIds.forEach(id => document.getElementById(id).classList.add('hidden'));
}

export function showNotification(message) {
    const notification = document.createElement('div');
    notification.textContent = message;

    notification.style.position = 'fixed';
    notification.style.bottom = '10px';
    notification.style.right = '10px';
    notification.style.background = 'rgba(79, 143, 43, 0.5)';
    notification.style.padding = '20px';
    notification.style.fontSize = '20px';
    notification.style.minWidth = '200px';
    notification.style.border = '1px solid #333';

    document.body.appendChild(notification);

    setTimeout(() => {
        notification.remove();
    }, 3000);
}


export function showCenterNotification(message, duration = 3000) {
    
    const notification = document.createElement('div');
    notification.textContent = message;

    Object.assign(notification.style, {
        position: 'fixed',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
        backgroundColor: 'rgba(0, 0, 0, 0.8)',
        color: 'white',
        padding: '20px 40px',
        borderRadius: '10px',
        boxShadow: '0 5px 15px rgba(0, 0, 0, 0.5)',
        textAlign: 'center',
        fontSize: '18px',
        zIndex: '10000', 
    });

    document.body.appendChild(notification);

    setTimeout(() => {
        notification.remove();
    }, duration);
}
