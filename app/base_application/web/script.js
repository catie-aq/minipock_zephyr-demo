$(document).ready(function () {
  // Handle navigation
  $(".list-group-item").click(function (e) {
    e.preventDefault();

    // Update active states
    $(".list-group-item").removeClass("active");
    $(this).addClass("active");

    // Show selected section
    $(".content-section").removeClass("active");
    $($(this).attr("href")).addClass("active");
  });
});

$(document).ready(function () {
  // Show confirmation modal on restart button click
  $("#restartButton").click(function () {
    $("#restartModal").modal("show");
  });

  // Handle confirm restart
  $("#confirmRestartButton").click(function () {
    // Add actual restart logic here
    console.log("System restart confirmed");
    $("#restartModal").modal("hide");
  });
});

document.getElementById("dhcpEnabled").addEventListener("change", function () {
  const staticIp = document.getElementById("staticIp");
  const subnetMask = document.getElementById("subnetMask");
  const gateway = document.getElementById("gateway");
  const updateButton = document.getElementById("updateIpButton");

  staticIp.disabled = this.checked;
  subnetMask.disabled = this.checked;
  gateway.disabled = this.checked;
});

document
  .getElementById("checkUpdateButton")
  .addEventListener("click", function () {
    const updateStatus = document.getElementById("updateStatus");
    const checkButton = document.getElementById("checkUpdateButton");

    // Show checking status
    updateStatus.innerHTML =
      '<span class="badge bg-info">Checking for updates...</span>';

    // Simulate version check (replace with actual version check)
    setTimeout(() => {
      const currentVersion = "1.0.0";
      const latestVersion = "1.1.0";

      if (latestVersion > currentVersion) {
        // Update available
        updateStatus.innerHTML = `<span class="badge bg-warning">Update available: v${latestVersion}</span>`;
        checkButton.innerHTML =
          '<i class="bi bi-github me-2"></i>Update from GitHub';
        checkButton.classList.remove("btn-info");
        checkButton.classList.add("btn-warning");
      } else {
        // No update needed
        updateStatus.innerHTML =
          '<span class="badge bg-success">System is up to date</span>';
        checkButton.innerHTML =
          '<i class="bi bi-search me-2"></i>Check for Updates';
        checkButton.classList.remove("btn-primary");
        checkButton.classList.add("btn-info");
      }
    }, 1000); // Simulated delay for the check
  });

document.getElementById("estopButton").addEventListener("click", function () {
  const button = this;
  const isActive = button.classList.contains("btn-danger");

  if (isActive) {
    // E-Stop Released
    button.classList.remove("btn-danger");
    button.classList.add("btn-success");
    button.style.animation = "";
  } else {
    // E-Stop Activated
    button.classList.remove("btn-success");
    button.classList.add("btn-danger");
    button.style.animation = "pulse 2s infinite";
  }
});
