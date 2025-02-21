$(document).ready(function() {
    // Handle navigation
    $('.list-group-item').click(function(e) {
        e.preventDefault();

        // Update active states
        $('.list-group-item').removeClass('active');
        $(this).addClass('active');

        // Show selected section
        $('.content-section').removeClass('active');
        $($(this).attr('href')).addClass('active');
    });
});

$(document).ready(function() {
    // Show confirmation modal on restart button click
    $('#restartButton').click(function() {
        $('#restartModal').modal('show');
    });

    // Handle confirm restart
    $('#confirmRestartButton').click(function() {
        // Add actual restart logic here
        console.log('System restart confirmed');
        $('#restartModal').modal('hide');
    });
});

document.getElementById('dhcpEnabled').addEventListener('change', function() {
    const staticIp = document.getElementById('staticIp');
    const subnetMask = document.getElementById('subnetMask');
    const gateway = document.getElementById('gateway');
    const updateButton = document.getElementById('updateIpButton');
    
    staticIp.disabled = this.checked;
    subnetMask.disabled = this.checked;
    gateway.disabled = this.checked;
});