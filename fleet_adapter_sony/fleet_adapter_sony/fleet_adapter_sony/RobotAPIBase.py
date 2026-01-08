from __future__ import annotations

from abc import ABC, abstractmethod
import time
import threading
from typing import Optional, Any, Dict, List, Tuple

import requests
from requests.exceptions import ConnectionError, HTTPError, Timeout, RequestException


# =============================================================================
# Robot API Base
# =============================================================================
class RobotAPIBase(ABC):
    def __init__(self, prefix: str, username: str, password: str):
        self.prefix = prefix
        self.username = username
        self.password = password
        self.proxies = {
            "http": "",
            "https": "",
        }
        self.connected = False

    # -------------------------------------------------------------------------
    # Connectivity
    # -------------------------------------------------------------------------
    @abstractmethod
    def check_connection(self) -> bool:
        """サーバとの接続確認

        Returns:
            bool: サーバとの接続成功可否
        """
        raise NotImplementedError

    # -------------------------------------------------------------------------
    # Robot state (read)
    # -------------------------------------------------------------------------
    @abstractmethod
    def get_position(self, robot_name: str) -> Optional[List[float]]:
        """ロボットの位置情報取得

        Args:
            robot_name (str): ロボット名

        Returns:
            Optional[List[float]]: [x, y, theta]（単位は各社APIの仕様に従う）
        """
        raise NotImplementedError

    @abstractmethod
    def get_status(self, robot_name: str) -> int:
        """ロボットの状態取得

        Args:
            robot_name (str): ロボット名

        Returns:
            int: 各社定義のステータスコード
        """
        raise NotImplementedError

    @abstractmethod
    def battery_soc(self, robot_name: str) -> Optional[float]:
        """ロボットのバッテリ状態取得

        Args:
            robot_name (str): ロボット名

        Returns:
            Optional[float]: 0.0〜1.0（取得できない場合は None）
        """
        raise NotImplementedError

    # -------------------------------------------------------------------------
    # Robot control (pause/resume)
    # -------------------------------------------------------------------------
    @abstractmethod
    def pause(self, robot_name: str) -> bool:
        """ロボットに一時停止指令

        Args:
            robot_name (str): ロボット名

        Returns:
            bool: 成功可否
        """
        raise NotImplementedError

    @abstractmethod
    def resume(self, robot_name: str) -> bool:
        """ロボットに再開指令

        Args:
            robot_name (str): ロボット名

        Returns:
            bool: 成功可否
        """
        raise NotImplementedError

    # -------------------------------------------------------------------------
    # Full-control primitives (navigate/stop)
    #
    # テンプレート (RobotCommandHandle / fleet_adapter) では、full_control の
    # 入口として以下のAPIが呼ばれます。各社の外部APIの違いが大きいので、
    # ここでは「RMFから見た最小限の意味」を揃えることを目的にしています。
    # -------------------------------------------------------------------------
    @abstractmethod
    def navigate(self, robot_name: str, destination: List[float], map_name: str, *, waypoint_name: Optional[str] = None, angle_rad: Optional[float] = None) -> bool:
        """目的地への移動指令

        Args:
            robot_name: ロボット名
            destination: [x, y, theta]（単位は各社APIの仕様に従う）
            map_name: 地図名

        Returns:
            bool: 受理可否
        """
        raise NotImplementedError

    @abstractmethod
    def stop(self, robot_name: str) -> bool:
        """停止指令

        Returns:
            bool: 成功可否
        """
        raise NotImplementedError

    @abstractmethod
    def navigation_remaining_duration(self, robot_name: str) -> float:
        """目的地到達までの残り時間（秒）を返す

        NOTE:
            外部APIに「残り時間」が無い場合は推定値でよい。
        """
        raise NotImplementedError

    @abstractmethod
    def navigation_completed(self, robot_name: str) -> bool:
        """直近の移動指令が完了したか"""
        raise NotImplementedError

    # -------------------------------------------------------------------------
    # Optional: process/docking primitives
    # -------------------------------------------------------------------------
    def start_process(self, robot_name: str, process: str, map_name: str) -> bool:
        """任意のプロセス開始（デリバリのロード/アンロード等）

        各社APIが存在しない場合は False を返す実装でよい。
        """
        return False

    def docking_completed(self, robot_name: str) -> bool:
        """ドッキング/プロセスが完了したか

        各社APIが存在しない場合は True を返す実装でよい（= no-op）。
        """
        return True


# =============================================================================
# RMF Web API (Keycloak token + REST) helper
# =============================================================================
class RMFAPI:
    def __init__(
        self,
        prefix: str,
        keycloak_url: str,
        username: str,
        password: str
    ):
        self.prefix = prefix
        self.keycloak_url = keycloak_url
        self.username = username
        self.password = password
        self.proxies = {
            "http": "",
            "https": "",
        }
        self.timeout: Tuple[float, float] = (3.0, 7.0)
        self.token: Optional[Dict[str, Any]] = None

        self.interval = 10
        self.is_running = False
        self.timer: Optional[threading.Timer] = None

        self.connected = False
        # Test connectivity
        connected = self.check_connection()
        if connected:
            print("Successfully able to query Keycloak server")
            self.connected = True
            self.start()
        else:
            print("Unable to query Keycloak server")

    def start(self) -> None:
        if not self.is_running:
            self.is_running = True
            self.timer = threading.Timer(self.interval, self.run)
            self.timer.start()

    def run(self) -> None:
        try:
            if self.token is not None and self.is_token_expired_in(30):
                if self.refresh_token():
                    print("token complete refresh")
                time.sleep(1)
            else:
                time.sleep(2)
        finally:
            self.is_running = False
            self.start()

    def _token_url(self) -> str:
        return self.keycloak_url + "/realms/rmf-web/protocol/openid-connect/token"

    def get_token(self) -> bool:
        """Keycloakのトークン取得関数

        Returns:
            bool: Keycloakトークンの取得成功可否
        """
        url = self._token_url()
        data = {
            # NOTE: 仕様上は grant_type
            "grant_type": "password",
            "client_id": "adapter",
            "username": self.username,
            "password": self.password,
        }
        try:
            response = requests.post(url=url, data=data, proxies=self.proxies, timeout=self.timeout)
            response.raise_for_status()
            token = response.json()
            token["acquired_at"] = time.time()
            token["expiration_time"] = time.time() + float(token.get("expires_in", 0))
            self.token = token
            return True
        except ConnectionError as connect_err:
            print(f"Connection error: {connect_err}")
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Timeout as timeout_err:
            print(f"Timeout error: {timeout_err}")
        except RequestException as req_err:
            print(f"Request error: {req_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def refresh_token(self) -> bool:
        """Keycloakのトークン更新関数

        Returns:
            bool: Keycloakトークンの更新成功可否
        """
        if self.token is None:
            return self.get_token()

        url = self._token_url()
        data = {
            # NOTE: 仕様上は grant_type
            "grant_type": "refresh_token",
            "client_id": "adapter",
            "refresh_token": self.token.get("refresh_token", "")
        }
        try:
            response = requests.post(url=url, data=data, proxies=self.proxies, timeout=self.timeout)
            response.raise_for_status()
            token = response.json()
            token["acquired_at"] = time.time()
            token["expiration_time"] = time.time() + float(token.get("expires_in", 0))
            self.token = token
            return True
        except ConnectionError as connect_err:
            print(f"Connection error: {connect_err}")
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Timeout as timeout_err:
            print(f"Timeout error: {timeout_err}")
        except RequestException as req_err:
            print(f"Request error: {req_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def is_token_expired_in(self, sec: int) -> bool:
        """Keycloakトークンの有効期限を確認する関数

        Args:
            sec (int): 有効期限に対するマージンの秒数

        Returns:
            bool: 現在の時間が有効期限から指定した秒数以内か
        """
        if self.token is None:
            return True
        expiration_time = float(self.token.get("expiration_time", 0))
        current_time = time.time()
        return (expiration_time - current_time) < sec

    def check_connection(self) -> bool:
        """サーバとの初期接続を確認する関数

        Returns:
            bool: サーバとの接続成功可否
        """
        if not self.get_token():
            return False

        url = self.prefix + "/fleets"
        access_token = self.token["access_token"]  # type: ignore[index]
        headers = {
            "Authorization": f"Bearer {access_token}",
            "Content-Type": "application/json",
        }
        try:
            response = requests.get(url=url, headers=headers, proxies=self.proxies, timeout=self.timeout)
            response.raise_for_status()
            return response.status_code == 200
        except ConnectionError as connect_err:
            print(f"Connection error: {connect_err}")
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Timeout as timeout_err:
            print(f"Timeout error: {timeout_err}")
        except RequestException as req_err:
            print(f"Request error: {req_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def get_task_request(self, task_id: str) -> Optional[Dict[str, Any]]:
        """投入されたタスクの詳細を取得する関数

        Args:
            task_id (str): RMFのタスクID

        Returns:
            Optional[Dict[str, Any]]: RMFタスクの詳細（取得失敗時は None）
        """
        if self.token is None:
            if not self.get_token():
                return None

        url = self.prefix + f"/tasks/{task_id}/request"
        access_token = self.token["access_token"]  # type: ignore[index]
        headers = {
            "Authorization": f"Bearer {access_token}",
            "Content-Type": "application/json",
        }
        try:
            response = requests.get(url=url, headers=headers, proxies=self.proxies, timeout=self.timeout)
            response.raise_for_status()
            if response.status_code == 200:
                return response.json()
            return None
        except ConnectionError as connect_err:
            print(f"Connection error: {connect_err}")
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Timeout as timeout_err:
            print(f"Timeout error: {timeout_err}")
        except RequestException as req_err:
            print(f"Request error: {req_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None
