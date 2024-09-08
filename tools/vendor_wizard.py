import os
import json
import requests
from packaging import version
import customtkinter as ctk
from customtkinter import CTkScrollableFrame
from tkinter import messagebox
import threading
import subprocess

# Change to the parent directory (from 'tools' to the root folder)
os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def check_internet():
    try:
        subprocess.check_output(["ping", "-c", "1", "8.8.8.8"], stderr=subprocess.STDOUT)
        return True
    except subprocess.CalledProcessError:
        return False

def check_vendordep_update(local_json_path, online_json_url):
    try:
        with open(local_json_path, 'r') as file:
            local_data = json.load(file)
        response = requests.get(online_json_url)
        if response.status_code == 200:
            online_data = response.json()
        else:
            return None, f"Failed to fetch online JSON. Status code: {response.status_code}"
        local_version = version.parse(local_data['version'])
        online_version = version.parse(online_data['version'])
        if online_version > local_version:
            return (local_version, online_version), None
        else:
            return None, None
    except Exception as e:
        return None, str(e)

def update_all_vendordeps():
    vendordeps_folder = os.path.join(os.getcwd(), 'vendordeps')
    for file_name in os.listdir(vendordeps_folder):
        if file_name == 'WPILibNewCommands.json':
            continue  # Skip this specific file as it can not be updated
        if file_name.endswith('.json'):
            local_json_path = os.path.join(vendordeps_folder, file_name)
            with open(local_json_path, 'r') as file:
                local_data = json.load(file)
            online_json_url = local_data['jsonUrl']
            result, error = check_vendordep_update(local_json_path, online_json_url)
            if result:
                update_vendordep(local_json_path, online_json_url)
    refresh_vendordeps()

def update_vendordeps_ui():
    loading_label.pack(pady=5)
    loading_label.configure(text="Refreshing Vendordeps")
    root.update_idletasks()

    for widget in scrollable_frame.winfo_children():
        widget.destroy()

    outdated_count = 0
    total_count = 0
    vendordeps_folder = os.path.join(os.getcwd(), 'vendordeps')

    if not check_internet():
        messagebox.showerror("Error", "No internet connection. Showing current vendordep versions only.")
        for file_name in os.listdir(vendordeps_folder):
            if file_name == 'WPILibNewCommands.json':
                continue  # Skip this specific file as it can not be updated
            if file_name.endswith('.json'):
                local_json_path = os.path.join(vendordeps_folder, file_name)
                with open(local_json_path, 'r') as file:
                    local_data = json.load(file)

                frame = ctk.CTkFrame(scrollable_frame)
                frame.pack(fill="x", padx=10, pady=5)

                info_frame = ctk.CTkFrame(frame, fg_color="transparent")
                info_frame.pack(side="left", fill="x", expand=True)

                ctk.CTkLabel(info_frame, text=f"{local_data['name']}", font=("Arial", 14, "bold"), anchor="w").pack(fill="x")
                version_frame = ctk.CTkFrame(info_frame, fg_color="transparent")
                version_frame.pack(fill="x")

                total_count += 1
                ctk.CTkLabel(version_frame, text=str(local_data['version']), text_color="yellow", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=" ⚠", text_color="yellow", anchor="w").pack(side="left")

        loading_label.pack_forget()
        refresh_button.configure(state="normal")
        update_all_button.configure(state="disabled")
        info_label.configure(text=f"Loaded {total_count} vendordeps. No internet", text_color="yellow")
        return

    for file_name in os.listdir(vendordeps_folder):
        if file_name == 'WPILibNewCommands.json':
            continue  # Skip this specific file as it can not be updated
        if file_name.endswith('.json'):
            local_json_path = os.path.join(vendordeps_folder, file_name)
            with open(local_json_path, 'r') as file:
                local_data = json.load(file)
            online_json_url = local_data['jsonUrl']
            result, error = check_vendordep_update(local_json_path, online_json_url)

            frame = ctk.CTkFrame(scrollable_frame)
            frame.pack(fill="x", padx=10, pady=5)

            info_frame = ctk.CTkFrame(frame, fg_color="transparent")
            info_frame.pack(side="left", fill="x", expand=True)

            ctk.CTkLabel(info_frame, text=f"{local_data['name']}", font=("Arial", 14, "bold"), anchor="w").pack(fill="x")
            version_frame = ctk.CTkFrame(info_frame, fg_color="transparent")
            version_frame.pack(fill="x")

            total_count += 1

            if error:
                ctk.CTkLabel(version_frame, text="Error: ", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=error, text_color="red", anchor="w").pack(side="left")
            elif result:
                local_version, online_version = result
                ctk.CTkLabel(version_frame, text="Current: ", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=str(local_version), text_color="red", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=" | Latest: ", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=str(online_version), text_color="green", anchor="w").pack(side="left")

                update_button = ctk.CTkButton(frame, text="Update", width=60,
                                              command=lambda p=local_json_path, u=online_json_url: update_vendordep(p, u))
                update_button.pack(side="right", padx=(10, 0))

                outdated_count += 1
            else:
                ctk.CTkLabel(version_frame, text=str(local_data['version']), text_color="green", anchor="w").pack(side="left")
                ctk.CTkLabel(version_frame, text=" ✔", text_color="green", anchor="w").pack(side="left")

    loading_label.pack_forget()
    refresh_button.configure(state="normal")
    update_all_button.configure(state="normal" if outdated_count > 0 else "disabled")

    if outdated_count > 0:
        info_label.configure(text=f"{outdated_count} vendordeps outdated", text_color="red")
    else:
        info_label.configure(text=f"All {total_count} vendordeps up to date!", text_color="green")

def show_success_popup(message):
    popup = ctk.CTkToplevel(root)
    popup.title("Success")
    popup.geometry("300x150")
    popup.configure(fg_color="gray20")

    message_with_emoji = f"✅ {message}"

    label = ctk.CTkLabel(popup, text=message_with_emoji, font=("Arial", 14),
                          fg_color="gray20", text_color="white", wraplength=250, anchor="center")
    label.pack(pady=20)

    ok_button = ctk.CTkButton(popup, text="OK", command=popup.destroy)
    ok_button.pack(pady=(0, 20))

    popup.transient(root)
    popup.grab_set()
    root.wait_window(popup)

def refresh_vendordeps():
    loading_label.pack(pady=5)
    loading_label.configure(text="Refreshing Vendordeps")
    refresh_button.configure(state="disabled")
    threading.Thread(target=update_vendordeps_ui, daemon=True).start()
    loading_label.pack_forget()

def update_vendordep(local_json_path, online_json_url):
    try:
        response = requests.get(online_json_url)
        if response.status_code == 200:
            with open(local_json_path, 'w') as file:
                file.write(response.text)
            show_success_popup(f"Updated {os.path.basename(local_json_path)} successfully!")
            update_vendordeps_ui()
        else:
            messagebox.showerror("Error", f"Failed to fetch online JSON. Status code: {response.status_code}")
    except Exception as e:
        messagebox.showerror("Error", str(e))
    refresh_vendordeps()

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

root = ctk.CTk()
root.title("Vendordep Wizard 🧙")
root.geometry("500x400")

top_frame = ctk.CTkFrame(root)
top_frame.pack(fill="x")

info_label = ctk.CTkLabel(top_frame, text="Waiting for data", anchor="w")
info_label.pack(side="left", padx=10, pady=10)

button_frame = ctk.CTkFrame(top_frame, fg_color="transparent")
button_frame.pack(side="right", padx=10, pady=10)

update_all_button = ctk.CTkButton(button_frame, text="Update All", command=update_all_vendordeps)
update_all_button.pack(side="left", padx=(0, 10))

refresh_button = ctk.CTkButton(button_frame, text="Refresh", command=refresh_vendordeps)
refresh_button.pack(side="left")

scrollable_frame = CTkScrollableFrame(root)
scrollable_frame.pack(fill="both", expand=True, padx=10, pady=10)

loading_label = ctk.CTkLabel(root, text="")

update_vendordeps_ui()

root.mainloop()
