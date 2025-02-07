let camera, scene, renderer, controls;
let clickablePoints = [];
let pointLights = [];

// Initialize immediately since we're not using modules
init();
animate();

function init() {
    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.1, 1000);
    
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x1e1e1e);
    renderer.shadowMap.enabled = true;
    document.getElementById('scene-container').appendChild(renderer.domElement);

    // Add lights
    const ambientLight = new THREE.AmbientLight(0xdd9fed, .5);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(5, 5, 5);
    directionalLight.castShadow = true;
    scene.add(directionalLight);

    // Set up camera and controls
    camera.position.set(0, 25, 8);
    try {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.maxDistance = 4;
        controls.minDistance = 3;
        controls.autoRotate = false;
        controls.autoRotateSpeed = 0.5;
        controls.target.set(0, 0, 0);
    } catch (e) {
        console.error('Error creating controls:', e);
    }


    // Load the reef model
    try {
        const loader = new THREE.GLTFLoader();
        loader.load('reef.glb', 
            function(gltf) {
                const model = gltf.scene;
                model.traverse(function(node) {
                    if (node.isMesh) {
                        node.castShadow = true;
                        node.receiveShadow = true;
                    }
                });
                scene.add(model);
                addClickablePoints();
                addFloorText(); 
            },
            function(xhr) {
                const loadingProgress = (xhr.loaded / xhr.total * 100);
                console.log(loadingProgress + '% loaded');
                // You could add a loading indicator here
            },
            function(error) {
                console.error('Error loading model:', error);
            }
        );
    } catch (e) {
        console.error('Error setting up loader:', e);
    }

    window.addEventListener('resize', onWindowResize, false);
    window.addEventListener('click', onMouseClick, false);
    
    // Add hover effect
    window.addEventListener('mousemove', onMouseMove, false);
}

function createPointMesh() {
    // Create group to hold both sphere and outline
    const group = new THREE.Group();

    // Inner sphere (white)
    const innerGeometry = new THREE.SphereGeometry(0.08);
    const innerMaterial = new THREE.MeshStandardMaterial({ 
        color: 0xffffff,
        metalness: 0.3,
        roughness: 0.4
    });
    const innerSphere = new THREE.Mesh(innerGeometry, innerMaterial);
    group.add(innerSphere);

    // Outer sphere (black outline)
    const outerGeometry = new THREE.SphereGeometry(0.1);
    const outerMaterial = new THREE.MeshBasicMaterial({ 
        color: 0x000000,
        side: THREE.BackSide
    });
    const outerSphere = new THREE.Mesh(outerGeometry, outerMaterial);
    group.add(outerSphere);

    // Add point light (initially invisible)
    const pointLight = new THREE.PointLight(0x00ff00, 0, 1);
    pointLight.intensity = 0;
    group.add(pointLight);
    pointLights.push(pointLight);

    return group;
}

function addClickablePoints() {
    const positions = [
        // X (distance from center), Y (distance from bottom of reef), X (distance from center, left/right)

        // TOP FACE OF REEF HEXAGON
        { x: .75, y: .7, z: .15, command: 'TOP_LEFT_L2' },
        { x: .75, y: .7, z: -.15, command: 'TOP_RIGHT_L2' },

        { x: .75, y: 1.1, z: .15, command: 'TOP_LEFT_L3' },
        { x: .75, y: 1.1, z: -.15, command: 'TOP_RIGHT_L3' },

        { x: .78, y: 1.68, z: .15, command: 'TOP_LEFT_L4' },
        { x: .78, y: 1.68, z: -.15, command: 'TOP_RIGHT_L4'},

        // TOP LEFT FACE OF REEF HEXAGON
        { x: .57, y: .7, z: -.58, command: 'TOP_LEFT_TOP_L2' },
        { x: .23, y: .7, z: -.75, command: 'TOP_LEFT_BOTTOM_L2' },

        { x: .57, y: 1.1, z: -.58, command: 'TOP_LEFT_TOP_L3' },
        { x: .23, y: 1.1, z: -.75, command: 'TOP_LEFT_BOTTOM_L3' },

        { x: .57, y: 1.68, z: -.58, command: 'TOP_LEFT_TOP_L4' },
        { x: .23, y: 1.68, z: -.75, command: 'TOP_LEFT_BOTTOM_L4' },

        // TOP RIGHT FACE OF REEF HEXAGON

        { x: .57, y: .7, z: .58, command: 'TOP_RIGHT_TOP_L2' },
        { x: .23, y: .7, z: .75, command: 'TOP_RIGHT_BOTTOM_L2' },

        { x: .57, y: 1.1, z: .58, command: 'TOP_RIGHT_TOP_L3' },
        { x: .23, y: 1.1, z: .75, command: 'TOP_RIGHT_BOTTOM_L3' },

        { x: .57, y: 1.68, z: .58, command: 'TOP_RIGHT_TOP_L4' },
        { x: .23, y: 1.68, z: .75, command: 'TOP_RIGHT_BOTTOM_L4' },

        // BOTTOM FACE OF REEF HEXAGON
        { x: -.75, y: .7, z: .15, command: 'BOTTOM_LEFT_L2' },
        { x: -.75, y: .7, z: -.15, command: 'BOTTOM_RIGHT_L2' },

        { x: -.75, y: 1.1, z: .15, command: 'BOTTOM_LEFT_L3' },
        { x: -.75, y: 1.1, z: -.15, command: 'BOTTOM_RIGHT_L3' },

        { x: -.78, y: 1.68, z: .15, command: 'BOTTOM_LEFT_L4' },
        { x: -.78, y: 1.68, z: -.15, command: 'BOTTOM_RIGHT_L4'},


        // BOTTOM LEFT FACE OF REEF HEXAGON

        { x: -.57, y: .7, z: .58, command: 'BOTTOM_LEFT_TOP_L2' },
        { x: -.23, y: .7, z: .75, command: 'BOTTOM_LEFT_BOTTOM_L2' },

        { x: -.57, y: 1.1, z: .58, command: 'BOTTOM_LEFT_TOP_L3' },
        { x: -.23, y: 1.1, z: .75, command: 'BOTTOM_LEFT_BOTTOM_L3' },

        { x: -.57, y: 1.68, z: .58, command: 'BOTTOM_LEFT_TOP_L4' },
        { x: -.23, y: 1.68, z: .75, command: 'BOTTOM_LEFT_BOTTOM_L4' },


        // BOTTOM RIGHT FACE OF REEF HEXAGON

        { x: -.57, y: .7, z: -.58, command: 'TOP_LEFT_TOP_L2' },
        { x: -.23, y: .7, z: -.75, command: 'BOTTOM_RIGHT_BOTTOM_L2' },

        { x: -.57, y: 1.1, z: -.58, command: 'BOTTOM_RIGHT_TOP_L3' },
        { x: -.23, y: 1.1, z: -.75, command: 'BOTTOM_RIGHT_BOTTOM_L3' },

        { x: -.57, y: 1.68, z: -.58, command: 'BOTTOM_RIGHT_TOP_L4' },
        { x: -.23, y: 1.68, z: -.75, command: 'BOTTOM_RIGHT_BOTTOM_L4' },
    ];

    positions.forEach(pos => {
        const point = createPointMesh();
        point.position.set(pos.x, pos.y, pos.z);
        point.userData.command = pos.command;
        point.userData.originalScale = new THREE.Vector3(1, 1, 1);
        scene.add(point);
        clickablePoints.push(point);
    });
}

function addFloorText() {
    const sides = ['TOP', 'BOTTOM', 'TOP LEFT', 'TOP RIGHT', 'BOTTOM LEFT', 'BOTTOM RIGHT'];
    const textMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.8 });

    sides.forEach((side, index) => {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 256;
        canvas.height = 128;
        context.font = 'bold 48px Arial';
        context.fillStyle = 'white';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(side, 128, 64);

        const texture = new THREE.CanvasTexture(canvas);
        const geometry = new THREE.PlaneGeometry(1, 0.5);
        const textMesh = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ map: texture, transparent: true }));

        // Rotate all text meshes to face upward
        textMesh.rotation.x = -Math.PI / 2;

        switch (side) {
            case 'TOP':
                textMesh.position.set(0, 0.01, 1.5);
                break;
            case 'BOTTOM':
                textMesh.position.set(0, 0.01, -1.5);
                textMesh.rotation.z = Math.PI;
                break;
            case 'TOP LEFT':
                textMesh.position.set(-1.3, 0.01, 0.75);
                textMesh.rotation.z = Math.PI / 3;
                break;
            case 'TOP RIGHT':
                textMesh.position.set(1.3, 0.01, 0.75);
                textMesh.rotation.z = -Math.PI / 3;
                break;
            case 'BOTTOM LEFT':
                textMesh.position.set(-1.3, 0.01, -0.75);
                textMesh.rotation.z = 2 * Math.PI / 3;
                break;
            case 'BOTTOM RIGHT':
                textMesh.position.set(1.3, 0.01, -0.75);
                textMesh.rotation.z = -2 * Math.PI / 3;
                break;
        }

        scene.add(textMesh);
    });
}


function onMouseMove(event) {
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
    
    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects(clickablePoints, true);
    
    // Reset all points
    clickablePoints.forEach(point => {
        point.scale.copy(point.userData.originalScale);
    });

    const tooltip = document.getElementById('tooltip');

    // Scale up hovered point and show tooltip
    if (intersects.length > 0) {
        const point = intersects[0].object.parent;
        point.scale.set(1.2, 1.2, 1.2);
        
        // Show tooltip with point ID
        tooltip.style.display = 'block';
        tooltip.style.left = event.clientX + 10 + 'px';
        tooltip.style.top = event.clientY + 10 + 'px';
        tooltip.textContent = point.userData.command;
    } else {
        // Hide tooltip when not hovering over a point
        tooltip.style.display = 'none';
    }
}


function onMouseClick(event) {
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
    
    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects(clickablePoints, true);
    
    if (intersects.length > 0) {
        const point = intersects[0].object.parent;
        const command = point.userData.command;
        
        // Visual feedback animation
        animateClick(point);
        
        // Send command
        sendCommand(command);
    }
}

function animateClick(point) {
    // Flash effect
    const light = point.children.find(child => child instanceof THREE.PointLight);
    if (light) {
        light.intensity = 2;
        setTimeout(() => {
            let startTime = Date.now();
            function fadeOut() {
                const elapsed = Date.now() - startTime;
                const duration = 500; // 500ms fade
                if (elapsed < duration) {
                    light.intensity = 2 * (1 - elapsed / duration);
                    requestAnimationFrame(fadeOut);
                } else {
                    light.intensity = 0;
                }
            }
            fadeOut();
        }, 100);
    }

    // Scale animation
    const originalScale = point.userData.originalScale;
    point.scale.set(1.5, 1.5, 1.5);
    setTimeout(() => {
        point.scale.copy(originalScale);
    }, 200);
}

function sendCommand(commandId) {
    fetch(`/command?id=${commandId}`)
        .then(response => {
            if (response.ok) {
                document.getElementById('last-command').textContent = commandId;
                // Add success feedback here if needed
            }
        })
        .catch(error => {
            console.error('Error:', error);
            // Add error feedback here if needed
        });
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    if (controls) {
        controls.update();
    }
    renderer.render(scene, camera);
}

// Connection status check
function checkConnection() {
    fetch('/status')
        .then(response => {
            const dot = document.querySelector('.indicator-dot');
            const text = document.querySelector('.indicator-text');
            if (response.ok) {
                dot.classList.add('connected');
                text.textContent = 'Connected';
            } else {
                dot.classList.remove('connected');
                text.textContent = 'Disconnected';
            }
        })
        .catch(() => {
            const dot = document.querySelector('.indicator-dot');
            const text = document.querySelector('.indicator-text');
            dot.classList.remove('connected');
            text.textContent = 'Disconnected';
        });
}

setInterval(checkConnection, 1000);
checkConnection();