from concurrent import futures
from dataclasses import dataclass

import grpc


@dataclass(frozen=True)
class GrpcServerConfig:
    port: int = 50051
    max_workers: int = 16

    # Streaming is used for large payloads, but thumbnails/CAD chunks may still be sizable.
    max_send_message_length: int = 50 * 1024 * 1024
    max_receive_message_length: int = 50 * 1024 * 1024


def start_server(pb2_grpc, servicer, cfg: GrpcServerConfig) -> grpc.Server:
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=cfg.max_workers),
        options=[
            ("grpc.max_send_message_length", int(cfg.max_send_message_length)),
            ("grpc.max_receive_message_length", int(cfg.max_receive_message_length)),
        ],
    )

    pb2_grpc.add_InspectionGatewayServicer_to_server(servicer, server)
    server.add_insecure_port(f"[::]:{int(cfg.port)}")
    server.start()
    return server

