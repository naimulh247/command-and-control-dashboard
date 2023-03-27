export function validateNum(input, min, max) {
    var num = +input;
    return num >= min && num <= max && input === num.toString();
}
