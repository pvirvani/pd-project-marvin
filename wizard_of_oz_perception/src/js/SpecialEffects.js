/***********************************************************
 * 
 * @param {*} scene 
 * @param {*} obj_name 
 * @param {*} opacity_ 
 **********************************************************/
export function changeOpacity(obj, opacity_) {
    // const obj = scene.getObjectByName(obj_name, true);
    console.log(obj);
    obj.traverse((child) => {
        if (child.isMesh && child.geometry !== undefined) {
            child.material.transparent = true;
            child.material.opacity = opacity_;
        }
    });
}